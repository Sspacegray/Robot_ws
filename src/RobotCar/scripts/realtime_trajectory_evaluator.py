#!/usr/bin/env python

import rospy
import tf
import os
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from rosbag import Bag
import datetime
import threading
import signal
import time
import subprocess
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult

class RealTimeTrajectoryEvaluator:
    def __init__(self):
        rospy.init_node('realtime_trajectory_evaluator', anonymous=True)
        
        # 创建输出目录
        self.output_dir = os.path.expanduser("~/trajectory_data")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # 初始化TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 初始化路径消息
        self.gt_path = Path()
        self.gt_path.header.frame_id = "map"
        
        self.est_path = Path()
        self.est_path.header.frame_id = "map"
        
        # 创建发布器，用于可视化
        self.gt_path_pub = rospy.Publisher('/ground_truth_path', Path, queue_size=1, latch=True)
        self.est_path_pub = rospy.Publisher('/estimated_path', Path, queue_size=1, latch=True)
        
        # 存储位姿数据
        self.gt_poses = []
        self.est_poses = []
        self.position_errors = []
        self.orientation_errors = []
        self.timestamps = []
        
        # 最近的位姿
        self.latest_gt_pose = None
        self.latest_est_pose = None
        self.latest_gt_time = None
        self.latest_est_time = None
        
        # 记录开始时间
        self.start_time = rospy.Time.now()
        self.bag_name = os.path.join(self.output_dir, f"trajectory_{self.start_time.to_sec():.0f}.bag")
        self.bag = Bag(self.bag_name, 'w')
        
        # 创建实时误差图表
        self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 12))
        self.position_line, = self.axes[0].plot([], [], 'r-')
        self.orientation_line, = self.axes[1].plot([], [], 'b-')
        
        self.axes[0].set_title('实时位置误差')
        self.axes[0].set_xlabel('时间 (秒)')
        self.axes[0].set_ylabel('位置误差 (米)')
        self.axes[0].grid(True)
        
        self.axes[1].set_title('实时方向误差')
        self.axes[1].set_xlabel('时间 (秒)')
        self.axes[1].set_ylabel('方向误差 (度)')
        self.axes[1].grid(True)
        
        # 设置图表更新定时器
        self.plot_timer = None
        
        # 订阅里程计消息
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 订阅AMCL位姿消息作为真实轨迹
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        
        # 订阅导航状态消息，用于检测导航结束
        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        self.move_base_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback)
        
        # 导航状态
        self.navigation_active = False
        self.navigation_completed = False
        
        # 定时器，每10秒保存一次路径
        self.save_timer = rospy.Timer(rospy.Duration(10), self.save_paths)
        
        # 启动图表更新线程
        self.plot_thread = threading.Thread(target=self.update_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()
        
        rospy.loginfo("实时轨迹评估器已启动，记录到: %s", self.bag_name)
        
    def odom_callback(self, msg):
        """处理里程计消息，作为估计轨迹"""
        # 创建PoseStamped消息
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # 添加到路径中
        self.est_path.header.stamp = rospy.Time.now()
        self.est_path.poses.append(pose_stamped)
        
        # 发布路径用于可视化
        self.est_path_pub.publish(self.est_path)
        
        # 将消息写入bag文件
        self.bag.write('/odom', msg)
        
        # 更新最新位姿
        self.latest_est_pose = msg.pose.pose
        self.latest_est_time = msg.header.stamp
        
        # 存储位姿数据
        self.est_poses.append((msg.header.stamp.to_sec(), msg.pose.pose))
        
        # 计算误差
        self.calculate_error()
        
    def amcl_pose_callback(self, msg):
        """处理AMCL位姿消息，作为真实轨迹"""
        # 创建PoseStamped消息
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # 添加到路径中
        self.gt_path.header.stamp = rospy.Time.now()
        self.gt_path.poses.append(pose_stamped)
        
        # 发布路径用于可视化
        self.gt_path_pub.publish(self.gt_path)
        
        # 将消息写入bag文件
        self.bag.write('/amcl_pose', msg)
        
        # 更新最新位姿
        self.latest_gt_pose = msg.pose.pose
        self.latest_gt_time = msg.header.stamp
        
        # 存储位姿数据
        self.gt_poses.append((msg.header.stamp.to_sec(), msg.pose.pose))
        
        # 计算误差
        self.calculate_error()
    
    def calculate_error(self):
        """计算当前位姿误差"""
        if self.latest_gt_pose is None or self.latest_est_pose is None:
            return
        
        # 计算位置误差
        gt_pos = self.latest_gt_pose.position
        est_pos = self.latest_est_pose.position
        position_error = np.sqrt((gt_pos.x - est_pos.x)**2 + 
                                 (gt_pos.y - est_pos.y)**2 + 
                                 (gt_pos.z - est_pos.z)**2)
        
        # 计算方向误差
        gt_quat = [self.latest_gt_pose.orientation.x, 
                   self.latest_gt_pose.orientation.y, 
                   self.latest_gt_pose.orientation.z, 
                   self.latest_gt_pose.orientation.w]
        
        est_quat = [self.latest_est_pose.orientation.x, 
                    self.latest_est_pose.orientation.y, 
                    self.latest_est_pose.orientation.z, 
                    self.latest_est_pose.orientation.w]
        
        # 计算四元数之间的角度差
        dot_product = np.abs(np.sum(np.array(gt_quat) * np.array(est_quat)))
        dot_product = min(1.0, max(-1.0, dot_product))  # 确保在[-1, 1]范围内
        orientation_error = 2 * np.arccos(dot_product) * 180.0 / np.pi  # 转换为角度
        
        # 存储误差数据
        current_time = rospy.Time.now().to_sec() - self.start_time.to_sec()
        self.timestamps.append(current_time)
        self.position_errors.append(position_error)
        self.orientation_errors.append(orientation_error)
        
        # 只保留最近的100个点用于显示
        if len(self.timestamps) > 100:
            self.timestamps.pop(0)
            self.position_errors.pop(0)
            self.orientation_errors.pop(0)
    
    def update_plot(self):
        """更新误差图表"""
        plt.ion()  # 开启交互模式
        
        while not rospy.is_shutdown():
            if len(self.timestamps) > 1:
                # 更新位置误差图
                self.position_line.set_data(self.timestamps, self.position_errors)
                self.axes[0].relim()
                self.axes[0].autoscale_view()
                
                # 更新方向误差图
                self.orientation_line.set_data(self.timestamps, self.orientation_errors)
                self.axes[1].relim()
                self.axes[1].autoscale_view()
                
                # 刷新图表
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
            
            time.sleep(0.5)  # 更新频率
    
    def move_base_status_callback(self, msg):
        """监控move_base状态"""
        if msg.status_list:
            # 检查是否有活动的导航目标
            self.navigation_active = any(status.status == 1 for status in msg.status_list)  # 1 = ACTIVE
    
    def move_base_result_callback(self, msg):
        """监控move_base结果"""
        # 检查导航是否完成
        if msg.status.status == 3:  # 3 = SUCCEEDED
            rospy.loginfo("导航目标已完成，准备生成评估报告")
            self.navigation_completed = True
            self.generate_evaluation_report()
    
    def save_paths(self, event):
        """定期保存路径"""
        # 将路径消息写入bag文件
        self.bag.write('/ground_truth_path', self.gt_path)
        self.bag.write('/estimated_path', self.est_path)
        
        rospy.loginfo("轨迹数据已保存到bag文件")
    
    def generate_evaluation_report(self):
        """生成评估报告"""
        # 关闭bag文件
        self.bag.close()
        
        # 创建结果目录
        results_dir = os.path.join(self.output_dir, "results", datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        if not os.path.exists(results_dir):
            os.makedirs(results_dir)
        
        # 计算统计指标
        if len(self.position_errors) > 0:
            position_metrics = {
                "mean": np.mean(self.position_errors),
                "median": np.median(self.position_errors),
                "std": np.std(self.position_errors),
                "min": np.min(self.position_errors),
                "max": np.max(self.position_errors),
                "rmse": np.sqrt(np.mean(np.array(self.position_errors) ** 2))
            }
            
            orientation_metrics = {
                "mean": np.mean(self.orientation_errors),
                "median": np.median(self.orientation_errors),
                "std": np.std(self.orientation_errors),
                "min": np.min(self.orientation_errors),
                "max": np.max(self.orientation_errors),
                "rmse": np.sqrt(np.mean(np.array(self.orientation_errors) ** 2))
            }
            
            # 保存指标到文件
            with open(os.path.join(results_dir, 'metrics.txt'), 'w') as f:
                f.write("=== 位置误差指标 ===\n")
                for key, value in position_metrics.items():
                    f.write(f"{key}: {value:.6f} m\n")
                
                f.write("\n=== 方向误差指标 ===\n")
                for key, value in orientation_metrics.items():
                    f.write(f"{key}: {value:.6f} 度\n")
            
            # 打印指标
            rospy.loginfo("\n=== 位置误差指标 ===")
            for key, value in position_metrics.items():
                rospy.loginfo(f"{key}: {value:.6f} m")
            
            rospy.loginfo("\n=== 方向误差指标 ===")
            for key, value in orientation_metrics.items():
                rospy.loginfo(f"{key}: {value:.6f} 度")
            
            # 保存图表
            # 轨迹图
            plt.figure(figsize=(10, 8))
            x_gt = [pose[1].position.x for pose in self.gt_poses]
            y_gt = [pose[1].position.y for pose in self.gt_poses]
            x_est = [pose[1].position.x for pose in self.est_poses]
            y_est = [pose[1].position.y for pose in self.est_poses]
            
            plt.plot(x_gt, y_gt, 'b-', label='Ground Truth (AMCL)')
            plt.plot(x_est, y_est, 'r-', label='Estimated (Odom)')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('Trajectory Comparison')
            plt.legend()
            plt.grid(True)
            plt.savefig(os.path.join(results_dir, 'trajectory.png'))
            
            # 位置误差图
            plt.figure(figsize=(10, 6))
            plt.plot(self.timestamps, self.position_errors)
            plt.xlabel('Time (s)')
            plt.ylabel('Position Error (m)')
            plt.title('Position Error')
            plt.grid(True)
            plt.savefig(os.path.join(results_dir, 'position_error.png'))
            
            # 方向误差图
            plt.figure(figsize=(10, 6))
            plt.plot(self.timestamps, self.orientation_errors)
            plt.xlabel('Time (s)')
            plt.ylabel('Orientation Error (degrees)')
            plt.title('Orientation Error')
            plt.grid(True)
            plt.savefig(os.path.join(results_dir, 'position_error_hist.png'))
            
            # 位置误差直方图
            plt.figure(figsize=(10, 6))
            plt.hist(self.position_errors, bins=30)
            plt.xlabel('Position Error (m)')
            plt.ylabel('Frequency')
            plt.title('Position Error Distribution')
            plt.grid(True)
            plt.savefig(os.path.join(results_dir, 'position_error_hist.png'))
            
            rospy.loginfo(f"评估报告已保存到: {results_dir}")
            
            # 可选：使用evaluate_trajectory.py脚本进行评估
            try:
                # 运行evaluate_trajectory.py脚本
                cmd = f"rosrun RobotCar evaluate_trajectory.py --bag {self.bag_name} --plot --save_results --dir {self.output_dir}"
                subprocess.Popen(cmd, shell=True)
                rospy.loginfo(f"已启动评估: {cmd}")
            except Exception as e:
                rospy.logerr(f"启动评估失败: {e}")
        else:
            rospy.logwarn("没有足够的数据生成评估报告")
    
    def shutdown(self):
        """关闭资源"""
        if hasattr(self, 'bag') and self.bag is not None:
            self.bag.close()
        
        # 如果导航未完成但程序被关闭，生成评估报告
        if not self.navigation_completed:
            self.generate_evaluation_report()
        
        plt.close('all')
        rospy.loginfo("实时轨迹评估器已关闭")

if __name__ == '__main__':
    try:
        evaluator = RealTimeTrajectoryEvaluator()
        
        # 注册信号处理函数，确保在Ctrl+C时正确关闭
        def signal_handler(sig, frame):
            rospy.loginfo("接收到关闭信号，正在关闭...")
            evaluator.shutdown()
            rospy.signal_shutdown("用户中断")
        
        signal.signal(signal.SIGINT, signal_handler)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 