include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "base_footprint",              -- 将所有传感器数据转换到这个坐标系下  --
  published_frame = "base_footprint",            -- tf: map -> footprint / tf_tree上最高的那一个坐标系
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = true,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint   用EKF的话就为false
  publish_frame_projected_to_2d = true,    -- 是否将坐标系投影到平面上
  use_pose_extrapolator = false,            -- 启用位姿外推器
  -- publish_tracked_pose = true,
  use_odometry = true,                     -- 是否使用里程计,如果使用要求一定要有odom的tf  使用EKF的odom
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  --上述中只允许同时订阅一个
  num_laser_scans = 1,                      -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据
  -- 上述中不允许同时为0

  -- 优化采样率配置
  rangefinder_sampling_ratio = 1.0,  -- 提高激光采样率
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,

    -- 时间相关参数优化
  lookup_transform_timeout_sec = 0.2,  -- 减少超时时间，加快重定位响应
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,  -- 提高发布频率
  trajectory_publish_period_sec = 0.1,  -- 提高发布频率
}
-- 导入的头文件库进行重写
MAP_BUILDER.use_trajectory_builder_2d = true  -- 使用2D建图

-- 纯定位模式配置
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,  -- 增加保留的子图数量
}

TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 使用imu数据
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0  -- 添加IMU重力常数配置，提高IMU数据利用
TRAJECTORY_BUILDER_2D.min_range = 0.15  -- 最小距离
TRAJECTORY_BUILDER_2D.max_range = 25.0  -- 最大距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- 适中的缺失数据处理值
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.0

--点云处理参数优化
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 思岚C1扫描频率较低，可以保持为1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.03  -- 点云滤波尺寸

-- 扫描匹配器参数优化
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3  -- 增大线性搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.)  -- 增大角度搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1


-- Ceres扫描匹配器参数优化
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 进一步增加占用空间权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0  -- 增加平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0  -- 进一步增加旋转权重

-- 自适应体素滤波优化
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100  -- 降低最小点数要求
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 25.0

-- 运动滤波器参数优化
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 40.0  -- 增加权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.0
--以下三个是调试建图效果的
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.  -- 占用空间权重
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.  --平移和先验偏差量的权重
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.  --旋转和先验偏差量的权重
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12


TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150  -- 增加子图中的点云数据量，提高子图质量
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 保持较高分辨率

POSE_GRAPH.optimize_every_n_nodes = 20  -- 更频繁地进行优化
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 提高采样率
POSE_GRAPH.global_sampling_ratio = 0.003  
POSE_GRAPH.constraint_builder.max_constraint_distance = 20.0  -- 增加最大约束距离
POSE_GRAPH.constraint_builder.min_score = 0.45  -- 降低最小得分阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45  -- 降低全局定位最小得分
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- 增加回环闭合平移权重
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.1e5  -- 增加回环闭合旋转权重
POSE_GRAPH.matcher_translation_weight = 5e2  -- 全局SLAM匹配器平移权重
POSE_GRAPH.matcher_rotation_weight = 1.6e3  -- 全局SLAM匹配器旋转权重
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- 增加Huber尺度，提高对异常值的鲁棒性
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3  -- 增加加速度权重
POSE_GRAPH.optimization_problem.rotation_weight = 3e5  -- 大幅增加旋转权重，减少角度漂移
POSE_GRAPH.optimization_problem.log_solver_summary = true  -- 输出求解器摘要，便于调试
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0  -- 添加IMU重力常数配置，提高IMU数据利用

-- 新增：全局定位相关参数
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0  -- 增大全局搜索范围
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(60.)  -- 增大角度搜索范围
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7  -- 增加分支定界深度
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_step = 0.1
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_step = math.rad(0.1)

return options  -- 返回配置信息




