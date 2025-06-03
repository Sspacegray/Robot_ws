include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "imu_link",              -- 将所有传感器数据转换到这个坐标系下  --
  published_frame = "base_footprint",            -- tf: map -> footprint / tf_tree上最高的那一个坐标系
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = true,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint   用EKF的话就为false
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上
  use_pose_extrapolator = false,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿
  -- publish_tracked_pose = true,
  publish_to_tf = true,                -- 是否发布tf
  use_odometry = true,                     -- 是否使用里程计,如果使用要求一定要有odom的tf  使用EKF的odom
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  --上述中只允许同时订阅一个
  num_laser_scans = 1,                      -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据
  -- 上述中不允许同时为0

  -- 添加采样率配置
  rangefinder_sampling_ratio = 0.8,
  odometry_sampling_ratio = 0.3,  -- 从0.05提高到0.3，增加里程计数据使用率
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,

  -- 增加时间相关参数
  lookup_transform_timeout_sec = 1.0,  -- 增加超时时间
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.1,  -- 提高到100ms，减少位姿更新频率
  trajectory_publish_period_sec = 0.1,  -- 提高发布频率
}
-- 导入的头文件库进行重写
MAP_BUILDER.use_trajectory_builder_2d = true  -- 使用2D建图

-- 纯定位
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 使用在线相关扫描匹配,计算量大但是很鲁棒，在odom或者imu不准时依然能达到很好的效果
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 使用imu数据
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 5.0  -- 减小IMU重力常数，改善IMU稳定性
TRAJECTORY_BUILDER_2D.min_range = 0.15  -- 最小距离
TRAJECTORY_BUILDER_2D.max_range = 8.0  -- 最大距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- 适中的缺失数据处理值
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.0

--点云处理参数
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1.5  -- 思岚C1扫描频率较低，可以保持为1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.03  -- 点云滤波尺寸

-- 扫描匹配器参数
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.35  -- 从0.15增加到0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(60.)  -- 从25度增加到45度
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5e-2  -- 减小权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5e-2  -- 减小权重


-- Ceres扫描匹配器参数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 占用空间权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.0  -- 平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 45.0  -- 旋转权重

-- 自适应体素滤波 - 适合思岚C1的点云密度
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150  -- 降低最小点数要求
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 8.0  -- 与最大范围一致

-- 重要！减少TF跳变的关键参数
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5  -- 增加时间阈值
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10  -- 从0.15减小
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(8.)  -- 从0.25减小

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 160  -- 增加子图中的点云数据量，提高子图质量
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 保持较高分辨率

POSE_GRAPH.optimize_every_n_nodes = 80  -- 更频繁地进行优化
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05   --后端优化 关掉局部约束生成即为0
POSE_GRAPH.global_sampling_ratio = 0.005  -- 从0.001增加到0.01，大幅提高全局采样率
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.0  -- 从15.0增加到25.0

POSE_GRAPH.constraint_builder.min_score = 0.65     --回环检测阈值，如果不稳定有错误匹配，可以提高这两个值，场景重复可以降低或者关闭回环
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65  -- 从0.6降低到0.50，提高重定位成功率

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- 保持不变
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.5e5  -- 保持不变
POSE_GRAPH.constraint_builder.log_matches = true -- 打印匹配结果 就是约束计算的log
POSE_GRAPH.global_constraint_search_after_n_seconds = 3. -- 从5秒减少到3秒，增加全局搜索频率
POSE_GRAPH.matcher_translation_weight = 5e2  -- 全局SLAM匹配器平移权重
POSE_GRAPH.matcher_rotation_weight = 1.6e3  -- 全局SLAM匹配器旋转权重
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- 增加Huber尺度，提高对异常值的鲁棒性
POSE_GRAPH.optimization_problem.acceleration_weight = 5e2  -- 减小加速度权重
POSE_GRAPH.optimization_problem.rotation_weight = 1e5  -- 减小旋转权重，减少角度抖动
POSE_GRAPH.optimization_problem.log_solver_summary = true  -- 输出求解器摘要，便于调试

-- 添加里程计权重参数
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2  -- 增加里程计平移权重
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2  -- 增加里程计旋转权重

-- 添加分支定界算法的关键参数
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0  -- 增加线性搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)  -- 增加角度搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth =  9 -- 增加搜索深度，默认为7

-- 添加重定位参数
POSE_GRAPH.max_num_final_iterations = 80  -- 最大迭代次数，提高优化精度

return options  -- 返回配置信息




