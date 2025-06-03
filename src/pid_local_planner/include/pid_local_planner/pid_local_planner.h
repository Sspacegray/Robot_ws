#ifndef PID_LOCAL_PLANNER_H
#define PID_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

namespace pid_local_planner {

/**
 * @class PIDLocalPlanner
 * @brief 一个简单的基于PID控制的局部规划器，专注于直线行驶和原地旋转
 */
class PIDLocalPlanner : public nav_core::BaseLocalPlanner {
public:
  /**
   * @brief 构造函数
   */
  PIDLocalPlanner();

  /**
   * @brief 析构函数
   */
  ~PIDLocalPlanner();

  /**
   * @brief 初始化规划器
   * @param name 规划器名称
   * @param tf 坐标变换缓冲区
   * @param costmap_ros 代价地图接口
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief 设置全局规划路径
   * @param global_plan 全局规划路径
   * @return 是否成功设置
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

  /**
   * @brief 计算下一步的速度指令
   * @param cmd_vel 输出的速度指令
   * @return 是否成功计算
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief 判断是否到达目标
   * @return 是否到达目标
   */
  bool isGoalReached();

private:
  // 规划器状态
  bool initialized_;
  bool goal_reached_;
  
  // 坐标变换
  tf2_ros::Buffer* tf_;
  std::string robot_base_frame_;
  
  // 代价地图
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  
  // 里程计
  base_local_planner::OdometryHelperRos odom_helper_;
  
  // 全局规划路径
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  
  // 当前目标点
  geometry_msgs::PoseStamped current_goal_;
  int current_goal_idx_;
  
  // PID控制器参数
  double linear_kp_, linear_ki_, linear_kd_;    // 线速度PID参数
  double angular_kp_, angular_ki_, angular_kd_; // 角速度PID参数
  
  // 积分项和前一次误差
  double linear_integral_, linear_previous_error_;
  double angular_integral_, angular_previous_error_;
  
  // 时间控制
  ros::Time last_update_time_;
  
  // 速度限制
  double max_vel_x_, min_vel_x_;
  double max_vel_theta_, min_vel_theta_;
  double min_in_place_vel_theta_;
  double acc_lim_x_, acc_lim_theta_;
  
  // 机器人特性
  bool holonomic_robot_;
  
  // 到达目标的条件
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  
  // 路径前向距离
  double lookahead_distance_;
  
  // 导航行为控制
  double rotation_threshold_;       // 执行原地旋转的阈值角度（弧度）
  double slowdown_distance_;        // 接近目标减速的距离阈值
  double obstacle_check_distance_;  // 检查障碍物的距离
  
  // 发布本地路径可视化
  ros::Publisher local_plan_pub_;
  
  // 路径跟踪相关函数
  bool getTargetPose(geometry_msgs::PoseStamped& target_pose);
  double calculateDistanceToObstacle(double angle);
  bool checkForObstacles();
  void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& path);
  void resetPID();
  
  // 处理目标函数
  bool shouldRotateToGoal(const geometry_msgs::PoseStamped& target_pose, double& angle_to_goal);
  bool shouldRotateToPath(const geometry_msgs::PoseStamped& target_pose, double& angle_to_path);
  double getPathAngle();
  
  // 获取机器人当前位姿
  bool getRobotPose(geometry_msgs::PoseStamped& robot_pose);
};

} // namespace pid_local_planner

#endif // PID_LOCAL_PLANNER_H
