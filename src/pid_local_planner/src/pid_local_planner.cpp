#include <pid_local_planner/pid_local_planner.h>
#include <pluginlib/class_list_macros.h>

// 声明这个类是一个基于nav_core::BaseLocalPlanner的插件
PLUGINLIB_EXPORT_CLASS(pid_local_planner::PIDLocalPlanner, nav_core::BaseLocalPlanner)

namespace pid_local_planner {

PIDLocalPlanner::PIDLocalPlanner() : initialized_(false), goal_reached_(false) {
}

PIDLocalPlanner::~PIDLocalPlanner() {
}

void PIDLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("PIDLocalPlanner already initialized");
    return;
  }
  
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  
  // 获取机器人基础帧的名字
  robot_base_frame_ = costmap_ros_->getBaseFrameID();
  
  // 初始化里程计辅助类
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);
  odom_helper_.setOdomTopic("odom");
  
  // 初始化发布者
  local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  
  // 获取参数
  private_nh.param("linear_kp", linear_kp_, 1.0);
  private_nh.param("linear_ki", linear_ki_, 0.0);
  private_nh.param("linear_kd", linear_kd_, 0.0);
  private_nh.param("angular_kp", angular_kp_, 1.5);
  private_nh.param("angular_ki", angular_ki_, 0.0);
  private_nh.param("angular_kd", angular_kd_, 0.1);
  
  private_nh.param("max_vel_x", max_vel_x_, 0.3);
  private_nh.param("min_vel_x", min_vel_x_, 0.0);
  private_nh.param("max_vel_theta", max_vel_theta_, 0.7);
  private_nh.param("min_vel_theta", min_vel_theta_, -0.7);
  private_nh.param("min_in_place_vel_theta", min_in_place_vel_theta_, 0.4);
  private_nh.param("acc_lim_x", acc_lim_x_, 0.8);
  private_nh.param("acc_lim_theta", acc_lim_theta_, 1.2);
  
  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
  private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);
  private_nh.param("lookahead_distance", lookahead_distance_, 0.5);
  private_nh.param("rotation_threshold", rotation_threshold_, 0.4);
  private_nh.param("slowdown_distance", slowdown_distance_, 0.5);
  private_nh.param("obstacle_check_distance", obstacle_check_distance_, 0.3);
  
  private_nh.param("holonomic_robot", holonomic_robot_, false);
  
  // 重置PID控制器
  resetPID();
  
  last_update_time_ = ros::Time::now();
  initialized_ = true;
  goal_reached_ = false;
  
  ROS_INFO("PIDLocalPlanner initialized");
}

bool PIDLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) {
  if (!initialized_) {
    ROS_ERROR("PIDLocalPlanner has not been initialized");
    return false;
  }
  
  // 保存全局路径
  global_plan_ = global_plan;
  
  // 重置目标达成状态
  goal_reached_ = false;
  
  // 重置PID控制器
  resetPID();
  
  // 重置目标索引
  current_goal_idx_ = 0;
  
  ROS_INFO("Global plan has been set with %zu points", global_plan_.size());
  return true;
}

bool PIDLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("PIDLocalPlanner has not been initialized");
    return false;
  }
  
  if (global_plan_.empty()) {
    ROS_ERROR("Global plan is empty");
    return false;
  }
  
  // 获取当前机器人位姿
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    ROS_ERROR("Failed to get robot pose");
    return false;
  }
  
  // 如果已经到达目标，发送零速度命令
  if (goal_reached_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }
  
  // 获取目标位姿
  geometry_msgs::PoseStamped target_pose;
  if (!getTargetPose(target_pose)) {
    ROS_ERROR("Failed to get target pose");
    return false;
  }
  
  // 检查是否到达最终目标
  double goal_dist = hypot(
    global_plan_.back().pose.position.x - robot_pose.pose.position.x,
    global_plan_.back().pose.position.y - robot_pose.pose.position.y
  );
  
  double yaw_diff = fabs(tf2::getYaw(global_plan_.back().pose.orientation) - tf2::getYaw(robot_pose.pose.orientation));
  yaw_diff = std::min(yaw_diff, 2 * M_PI - yaw_diff);
  
  if (goal_dist < xy_goal_tolerance_ && yaw_diff < yaw_goal_tolerance_) {
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    ROS_INFO("Goal reached");
    return true;
  }
  
  // 计算到目标的距离和角度
  double dx = target_pose.pose.position.x - robot_pose.pose.position.x;
  double dy = target_pose.pose.position.y - robot_pose.pose.position.y;
  double dist = hypot(dx, dy);
  
  // 计算机器人到目标的角度
  double target_yaw = atan2(dy, dx);
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double angle_diff = angles::shortest_angular_distance(robot_yaw, target_yaw);
  
  // 检查是否应该先旋转
  double angle_to_goal = 0.0;
  if (shouldRotateToGoal(target_pose, angle_to_goal) || 
      shouldRotateToPath(target_pose, angle_to_goal)) {
    // 仅执行旋转操作
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    
    // 计算角速度（PID控制）
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_update_time_).toSec();
    last_update_time_ = current_time;
    
    // 避免除零错误
    if (dt < 0.001) dt = 0.001;
    
    // 计算PID项
    double angular_error = angle_to_goal;
    angular_integral_ += angular_error * dt;
    double angular_derivative = (angular_error - angular_previous_error_) / dt;
    
    // 计算角速度
    double angular_vel = angular_kp_ * angular_error + angular_ki_ * angular_integral_ + angular_kd_ * angular_derivative;
    
    // 限制角速度范围
    angular_vel = std::max(min_vel_theta_, std::min(max_vel_theta_, angular_vel));
    
    // 确保旋转速度不会太小
    if (fabs(angular_vel) < min_in_place_vel_theta_ && fabs(angular_error) > 0.1) {
      angular_vel = angular_vel > 0 ? min_in_place_vel_theta_ : -min_in_place_vel_theta_;
    }
    
    cmd_vel.angular.z = angular_vel;
    angular_previous_error_ = angular_error;
    
    ROS_DEBUG("Rotating to path/goal, angle_diff: %.2f, cmd_vel: %.2f", angular_error, angular_vel);
    return true;
  }
  
  // 检查前方是否有障碍物
  if (checkForObstacles()) {
    ROS_WARN("Obstacle detected, stopping");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }
  
  // 计算线速度和角速度（PID控制）
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_update_time_).toSec();
  last_update_time_ = current_time;
  
  // 避免除零错误
  if (dt < 0.001) dt = 0.001;
  
  // 计算线速度PID项
  double linear_error = dist;
  linear_integral_ += linear_error * dt;
  double linear_derivative = (linear_error - linear_previous_error_) / dt;
  
  // 计算角速度PID项
  double angular_error = angle_diff;
  angular_integral_ += angular_error * dt;
  double angular_derivative = (angular_error - angular_previous_error_) / dt;
  
  // 计算线速度（根据距离调整）
  double linear_vel = linear_kp_ * linear_error + linear_ki_ * linear_integral_ + linear_kd_ * linear_derivative;
  
  // 根据距离减速
  if (dist < slowdown_distance_) {
    linear_vel *= (dist / slowdown_distance_);
  }
  
  // 根据角度差调整线速度
  linear_vel *= cos(angle_diff);
  
  // 限制线速度范围和加速度
  if (linear_vel > max_vel_x_) linear_vel = max_vel_x_;
  if (linear_vel < min_vel_x_) linear_vel = min_vel_x_;
  
  // 计算角速度
  double angular_vel = angular_kp_ * angular_error + angular_ki_ * angular_integral_ + angular_kd_ * angular_derivative;
  
  // 限制角速度范围
  if (angular_vel > max_vel_theta_) angular_vel = max_vel_theta_;
  if (angular_vel < min_vel_theta_) angular_vel = min_vel_theta_;
  
  // 保存误差用于下一次计算
  linear_previous_error_ = linear_error;
  angular_previous_error_ = angular_error;
  
  // 设置速度命令
  cmd_vel.linear.x = linear_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = angular_vel;
  
  ROS_DEBUG("Target: (%.2f, %.2f), Dist: %.2f, Angle: %.2f", 
            target_pose.pose.position.x, target_pose.pose.position.y, 
            dist, angle_diff);
  ROS_DEBUG("CMD: linear=%.2f, angular=%.2f", linear_vel, angular_vel);
  
  // 发布本地路径可视化
  std::vector<geometry_msgs::PoseStamped> local_plan;
  local_plan.push_back(robot_pose);
  local_plan.push_back(target_pose);
  publishLocalPlan(local_plan);
  
  return true;
}

bool PIDLocalPlanner::isGoalReached() {
  return goal_reached_;
}

bool PIDLocalPlanner::getTargetPose(geometry_msgs::PoseStamped& target_pose) {
  // 获取当前机器人位姿
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return false;
  }
  
  // 寻找前向点
  double min_lookahead_dist = lookahead_distance_ * 0.5;
  double lookahead_dist = lookahead_distance_;
  bool found_lookahead_point = false;
  
  // 从当前索引开始查找
  for (size_t i = current_goal_idx_; i < global_plan_.size(); i++) {
    double dist = hypot(
      global_plan_[i].pose.position.x - robot_pose.pose.position.x,
      global_plan_[i].pose.position.y - robot_pose.pose.position.y
    );
    
    // 找到离lookahead_distance最近的点
    if (dist >= min_lookahead_dist && dist <= lookahead_dist * 2.0) {
      target_pose = global_plan_[i];
      current_goal_idx_ = i;
      found_lookahead_point = true;
      break;
    }
    
    // 记录已经通过的点
    if (dist < min_lookahead_dist && i < global_plan_.size() - 1) {
      current_goal_idx_ = i;
    }
  }
  
  // 如果没有找到满足条件的点，使用最后一个点
  if (!found_lookahead_point) {
    if (current_goal_idx_ < global_plan_.size() - 1) {
      target_pose = global_plan_[current_goal_idx_ + 1];
    } else {
      target_pose = global_plan_.back();
    }
  }
  
  return true;
}

double PIDLocalPlanner::calculateDistanceToObstacle(double angle) {
  // 获取机器人位置
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return 0.0;
  }
  
  // 计算查询点的世界坐标
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double query_angle = robot_yaw + angle;
  
  // 在代价地图上检查从机器人向前的点
  double max_dist = obstacle_check_distance_;
  double resolution = costmap_->getResolution();
  int steps = static_cast<int>(max_dist / resolution);
  
  for (int i = 0; i < steps; i++) {
    double dist = i * resolution;
    double wx = robot_x + dist * cos(query_angle);
    double wy = robot_y + dist * sin(query_angle);
    
    // 将世界坐标转换为地图坐标
    unsigned int mx, my;
    if (costmap_->worldToMap(wx, wy, mx, my)) {
      unsigned char cost = costmap_->getCost(mx, my);
      
      // 检查是否为致命代价或者障碍物
      if (cost >= costmap_2d::LETHAL_OBSTACLE) {
        return dist;
      }
    }
  }
  
  return max_dist;
}

bool PIDLocalPlanner::checkForObstacles() {
  // 检查前方是否有障碍物
  double front_dist = calculateDistanceToObstacle(0.0);
  return front_dist < obstacle_check_distance_ * 0.5;
}

void PIDLocalPlanner::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
  if (!initialized_ || path.empty()) {
    return;
  }
  
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses = path;
  
  local_plan_pub_.publish(gui_path);
}

void PIDLocalPlanner::resetPID() {
  // 重置积分项和前一次误差
  linear_integral_ = 0.0;
  linear_previous_error_ = 0.0;
  angular_integral_ = 0.0;
  angular_previous_error_ = 0.0;
}

bool PIDLocalPlanner::shouldRotateToGoal(const geometry_msgs::PoseStamped& target_pose, double& angle_to_goal) {
  // 获取当前机器人位姿
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return false;
  }
  
  // 仅在接近最终目标时执行
  if (&target_pose != &global_plan_.back()) {
    double goal_dist = hypot(
      global_plan_.back().pose.position.x - robot_pose.pose.position.x,
      global_plan_.back().pose.position.y - robot_pose.pose.position.y
    );
    
    if (goal_dist > xy_goal_tolerance_ * 2.0) {
      return false;
    }
  }
  
  // 计算目标角度和机器人当前角度的差异
  double target_yaw = tf2::getYaw(target_pose.pose.orientation);
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  angle_to_goal = angles::shortest_angular_distance(robot_yaw, target_yaw);
  
  // 如果角度差大于阈值，需要先旋转
  return fabs(angle_to_goal) > rotation_threshold_;
}

bool PIDLocalPlanner::shouldRotateToPath(const geometry_msgs::PoseStamped& target_pose, double& angle_to_path) {
  // 获取当前机器人位姿
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return false;
  }
  
  // 计算到目标的向量
  double dx = target_pose.pose.position.x - robot_pose.pose.position.x;
  double dy = target_pose.pose.position.y - robot_pose.pose.position.y;
  
  // 计算路径角度和机器人当前角度的差异
  double path_angle = atan2(dy, dx);
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  angle_to_path = angles::shortest_angular_distance(robot_yaw, path_angle);
  
  // 如果角度差大于阈值，需要先旋转
  return fabs(angle_to_path) > rotation_threshold_;
}

double PIDLocalPlanner::getPathAngle() {
  // 如果路径太短，无法计算角度
  if (global_plan_.size() < 2) {
    return 0.0;
  }
  
  // 计算路径的方向（使用当前目标点和下一个点）
  size_t idx = (current_goal_idx_ < global_plan_.size() - 2) ? current_goal_idx_ : (global_plan_.size() - 2);
  double dx = global_plan_[idx + 1].pose.position.x - global_plan_[idx].pose.position.x;
  double dy = global_plan_[idx + 1].pose.position.y - global_plan_[idx].pose.position.y;
  
  return atan2(dy, dx);
}

bool PIDLocalPlanner::getRobotPose(geometry_msgs::PoseStamped& robot_pose) {
  if (!initialized_) {
    return false;
  }
  
  try {
    geometry_msgs::TransformStamped transform = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), robot_base_frame_, ros::Time(0)
    );
    
    robot_pose.header.frame_id = costmap_ros_->getGlobalFrameID();
    robot_pose.header.stamp = transform.header.stamp;
    
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation = transform.transform.rotation;
    
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("Failed to get robot pose: %s", ex.what());
    return false;
  }
}

} // namespace pid_local_planner
