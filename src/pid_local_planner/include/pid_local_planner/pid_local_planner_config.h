#ifndef PID_LOCAL_PLANNER_CONFIG_H
#define PID_LOCAL_PLANNER_CONFIG_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pid_local_planner/PIDLocalPlannerConfig.h>

namespace pid_local_planner {

/**
 * @class PIDLocalPlannerConfig
 * @brief 用于管理PID局部规划器的参数
 */
class PIDLocalPlannerConfig {
public:
  /**
   * @brief 构造函数
   * @param nh 节点句柄
   * @param pnh 私有节点句柄
   */
  PIDLocalPlannerConfig(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  /**
   * @brief 析构函数
   */
  ~PIDLocalPlannerConfig();

  /**
   * @brief 从ROS参数服务器加载参数
   */
  void loadParams();

  /**
   * @brief 动态重新配置参数的回调函数
   * @param config 新的配置
   * @param level 配置级别
   */
  void reconfigureCB(pid_local_planner::PIDLocalPlannerConfig& config, uint32_t level);

  // PID控制器参数
  double linear_kp;
  double linear_ki;
  double linear_kd;
  double angular_kp;
  double angular_ki;
  double angular_kd;

  // 速度限制
  double max_vel_x;
  double min_vel_x;
  double max_vel_theta;
  double min_vel_theta;
  double min_in_place_vel_theta;
  double acc_lim_x;
  double acc_lim_theta;

  // 目标容忍度
  double xy_goal_tolerance;
  double yaw_goal_tolerance;

  // 行为控制参数
  double lookahead_distance;
  double rotation_threshold;
  double slowdown_distance;
  double obstacle_check_distance;

  // 机器人配置
  bool holonomic_robot;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 动态重配置
  dynamic_reconfigure::Server<pid_local_planner::PIDLocalPlannerConfig> dsrv_;
};

} // namespace pid_local_planner

#endif // PID_LOCAL_PLANNER_CONFIG_H
