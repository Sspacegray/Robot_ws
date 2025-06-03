# PID局部规划器

## 简介

一个简化的基于PID控制的局部规划器，专为工厂巡检导航机器人设计。相比DWA局部规划器，本规划器更简单，参数更少，且更专注于直线行驶和原地旋转的行为模式。

## 特点

- 基于PID控制算法，易于调整和理解
- 参数少，易于配置
- 针对直线行驶和原地旋转优化
- 可通过动态重配置调整参数
- 与move_base框架完全兼容

## 参数说明

### PID控制器参数
- `linear_kp`：线速度P增益，控制线速度调整的灵敏度
- `linear_ki`：线速度I增益，消除累积误差
- `linear_kd`：线速度D增益，减少震荡
- `angular_kp`：角速度P增益
- `angular_ki`：角速度I增益
- `angular_kd`：角速度D增益

### 速度限制参数
- `max_vel_x`：最大线速度
- `min_vel_x`：最小线速度
- `max_vel_theta`：最大角速度
- `min_vel_theta`：最小角速度
- `min_in_place_vel_theta`：原地旋转最小角速度
- `acc_lim_x`：线加速度限制
- `acc_lim_theta`：角加速度限制

### 行为控制参数
- `lookahead_distance`：前视距离，控制规划器选择目标点的距离
- `rotation_threshold`：执行原地旋转的角度阈值（弧度）
- `slowdown_distance`：接近目标减速的距离
- `obstacle_check_distance`：障碍物检查距离

## 使用方法

1. 编译工作空间：
```
cd ~/Desktop/Robot_ws
catkin_make
source devel/setup.bash
```

2. 使用提供的启动文件测试：
```
roslaunch pid_local_planner test_pid_local_planner.launch
```

3. 整合到现有导航系统：
```
roslaunch pid_local_planner pid_navigation.launch
```

4. 或者只需在move_base启动文件中修改局部规划器：
```xml
<param name="base_local_planner" value="pid_local_planner/PIDLocalPlanner"/>
<rosparam file="$(find pid_local_planner)/param/pid_local_planner_params.yaml" command="load"/>
```

## 参数调优

- 对于较直的长距离路径，增加线速度P增益和最大线速度
- 对于复杂环境，降低线速度和前视距离
- 旋转控制对于通过门口至关重要，可以增加角速度P增益以提高旋转响应性

## 与DWA比较

相较于DWA局部规划器，本规划器：
- 参数更少，更易调整
- 针对工厂巡检场景优化，更好地处理直线和转弯
- 行为更可预测，不会在狭窄空间出现震荡
- 启动/停止/转弯的动作更加平滑
