/**
 * @file send_goals_node.cpp
 * @brief 多点导航循环运动程序（优化版）
 * * 主要优化点：
 * 1. 增强参数校验和错误处理
 * 2. 改进状态机逻辑确保运行稳定性
 * 3. 优化线程同步机制 (使用ros::Timer替代sleep)
 * 4. 完善导航状态管理
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <string>
#include <memory>
#include <boost/thread/mutex.hpp>
#include <XmlRpcValue.h> // 确保包含 XmlRpcValue

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint {
    double x;
    double y;
    double theta;
    std::string name;
};

class EnhancedNavigation {
public:
    EnhancedNavigation() :
        ac_(new MoveBaseClient("move_base", true)), // move_base action服务器名称硬编码为"move_base"
        is_active_(false),
        current_index_(0),
        loop_count_(0),
        total_loops_(0), // 默认为0，表示无限循环或由参数指定
        wait_seconds_at_waypoint_(3.0) // 默认等待时间
    {
        ros::NodeHandle nh; // 公共NodeHandle
        ros::NodeHandle private_nh("~"); // 私有NodeHandle，用于获取参数

        // 参数加载与校验
        if(!loadParameters(private_nh)) {
            ROS_ERROR("参数加载失败。");
            // 注意这里不再关闭节点，而是继续运行，取决于你的需求
        }

        // 服务接口 (只需要停止服务)
        stop_srv_ = nh.advertiseService("stop_navigation", &EnhancedNavigation::stopCallback, this);

        // 状态发布
        status_pub_ = nh.advertise<std_msgs::Bool>("navigation_status", 5);

        // 等待服务器连接
        ROS_INFO("等待move_base服务器连接...");
        if(!ac_->waitForServer(ros::Duration(10.0))) {
            ROS_ERROR("无法连接move_base服务器，节点关闭。");
            ros::shutdown();
            return;
        }
        ROS_INFO("move_base服务器已连接。");

        // 初始化定时器 (one_shot=true, auto_start=false)
        // 持续时间将在需要时设置
        wait_timer_ = nh.createTimer(ros::Duration(0.1), &EnhancedNavigation::waitTimerCallback, this, true, false);

        ROS_INFO("导航系统就绪，共加载%d个路径点。", (int)waypoints_.size());

        // 自动启动导航
        if (!waypoints_.empty()) {
            is_active_ = true;
            current_index_ = 0;
            loop_count_ = 0;
            ROS_INFO("自动启动导航任务。");
            sendNextGoal(); // 发送第一个目标点
        } else {
            ROS_WARN("没有加载到任何有效的路径点，导航不会自动启动。");
        }
    }

private:
    bool loadParameters(ros::NodeHandle& nh) {
        nh.param("total_loops", total_loops_, 0);
        ROS_INFO_STREAM("加载参数 'total_loops': " << total_loops_);

        nh.param("wait_seconds", wait_seconds_at_waypoint_, 3.0);
        ROS_INFO_STREAM("加载参数 'wait_seconds': " << wait_seconds_at_waypoint_);

        nh.param("frame_id", frame_id_, std::string("map"));
        ROS_INFO_STREAM("加载参数 'frame_id': " << frame_id_);

        XmlRpc::XmlRpcValue waypoints_list;
        if (!nh.getParam("waypoints", waypoints_list)) {
            ROS_WARN("未找到参数 'waypoints'。导航将不会包含任何目标点。");
            return true; // 没有路径点不是致命错误，可以继续
        }

        if (waypoints_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("参数 'waypoints' 不是一个列表 (Array)。导航将不会包含任何目标点。");
            return true; // 格式错误不是致命错误，可以继续
        }

        waypoints_.clear();
        if (waypoints_list.size() == 0) {
            ROS_INFO("参数 'waypoints' 列表为空。");
            return true;
        }

        ROS_INFO_STREAM("加载到 " << waypoints_list.size() << " 个路径点。");
        for (int i = 0; i < waypoints_list.size(); ++i) {
            XmlRpc::XmlRpcValue& waypoint_param = waypoints_list[i];
            if (waypoint_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN_STREAM("路径点 #" << i + 1 << " 不是一个结构体 (Struct)。已跳过该路径点。");
                continue;
            }

            if (!waypoint_param.hasMember("x") || !waypoint_param.hasMember("y")) {
                ROS_WARN_STREAM("路径点 #" << i + 1 << " 缺少必要的 'x' 或 'y' 字段。已跳过该路径点。");
                continue;
            }

            Waypoint waypoint;
            waypoint.x = static_cast<double>(waypoint_param["x"]);
            waypoint.y = static_cast<double>(waypoint_param["y"]);

            if (waypoint_param.hasMember("theta")) {
                waypoint.theta = static_cast<double>(waypoint_param["theta"]);
            } else {
                waypoint.theta = 0.0;
                ROS_INFO_STREAM("路径点 #" << i + 1 << " 缺少 'theta'，使用默认值 0.0。");
            }

            if (waypoint_param.hasMember("name")) {
                waypoint.name = static_cast<std::string>(waypoint_param["name"]);
            } else {
                waypoint.name = "point_" + std::to_string(i + 1);
                ROS_INFO_STREAM("路径点 #" << i + 1 << " 缺少 'name'，使用默认值 '" << waypoint.name << "'。");
            }
            waypoints_.push_back(waypoint);
        }

        if (waypoints_.empty()) {
            ROS_WARN("有效的路径点列表为空。");
            return true;
        }

        ROS_INFO("成功处理路径点参数。");
        return true;
    }

    bool stopCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        boost::mutex::scoped_lock lock(mutex_);

        if(is_active_) {
            ROS_INFO("收到停止导航任务指令。");
            ac_->cancelAllGoals();
            wait_timer_.stop(); // 停止等待定时器
            is_active_ = false;
            publishStatus(false);
            ROS_INFO("导航任务已终止。");
        } else {
            ROS_INFO("导航任务当前未激活，无需停止。");
        }
        return true;
    }

    void sendNextGoal() {
        // 此函数可能由 waitTimerCallback 调用，因此也需要锁
        // 或者确保调用它的地方（如startCallback, goalDone, waitTimerCallback）已经获取了锁
        // 当前设计：startCallback, goalDone, waitTimerCallback 都会获取锁，所以这里不需要重复获取

        if(!is_active_) {
            ROS_DEBUG("sendNextGoal 调用时 is_active_ 为 false，不发送目标。");
            return;
        }

        if(current_index_ >= waypoints_.size()) { // 完成了一轮路径点
            if(total_loops_ > 0) { // 如果 total_loops_ 设置为正数
                loop_count_++; // 完成的循环次数增加
                if(loop_count_ >= total_loops_) {
                    ROS_INFO("已完成预设的 %d 次导航循环。", total_loops_);
                    is_active_ = false;
                    wait_timer_.stop(); // 确保定时器停止
                    publishStatus(false);
                    return; // 所有循环完成
                }
            }
            // 如果 total_loops_ 为0，则无限循环，loop_count_ 会持续增加
            current_index_ = 0; // 重置到第一个路径点
            ROS_INFO("开始第 %d 次导航循环。", loop_count_ + 1); // loop_count_ 是已完成的次数
        }

        // 创建导航目标
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = frame_id_;

        const auto& wp = waypoints_[current_index_]; // 使用 const auto&
        goal.target_pose.pose.position.x = wp.x;
        goal.target_pose.pose.position.y = wp.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, wp.theta); // Roll, Pitch, Yaw
        goal.target_pose.pose.orientation = tf2::toMsg(q);

        ROS_INFO("循环 %d/%s, 前往路径点 [%s] (%zu/%zu): (%.2f, %.2f), 朝向 %.2f rad",
                 loop_count_ + 1,
                 (total_loops_ == 0 ? "无限" : std::to_string(total_loops_).c_str()),
                 wp.name.c_str(),
                 current_index_ + 1,
                 waypoints_.size(),
                 wp.x, wp.y, wp.theta);

        // 发送目标
        ac_->sendGoal(goal,
                         boost::bind(&EnhancedNavigation::goalDone, this, _1, _2),
                         boost::bind(&EnhancedNavigation::goalActive, this),
                         boost::bind(&EnhancedNavigation::goalFeedback, this, _1));
        publishStatus(true); // 更新状态为正在导航 (即使是刚发送)
    }

    void goalDone(const actionlib::SimpleClientGoalState& state,
                  const move_base_msgs::MoveBaseResultConstPtr& result) {
        boost::mutex::scoped_lock lock(mutex_);

        if(!is_active_) { // 如果在目标执行期间调用了stop
            ROS_DEBUG("目标完成回调时 is_active_ 为 false。");
            return;
        }

        std::string finished_waypoint_name = "未知路径点";
        if (current_index_ < waypoints_.size()) {
            finished_waypoint_name = waypoints_[current_index_].name;
        }

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("成功到达路径点 [%s]", finished_waypoint_name.c_str());
            current_index_++; // 指向下一个目标点的索引

            // 检查是否在最后一个路径点的最后一个循环（避免不必要的等待）
            // 这个检查也可以放在 sendNextGoal 的开头，目前设计中 sendNextGoal 会处理
            bool is_last_waypoint_of_last_loop = false;
            if (current_index_ >= waypoints_.size() && total_loops_ > 0 && (loop_count_ + 1) >= total_loops_) {
                is_last_waypoint_of_last_loop = true;
            }

            if (is_last_waypoint_of_last_loop) {
                ROS_INFO("到达最后一个路径点的最后一个循环，准备结束。");
                // 直接调用sendNextGoal，它会处理结束逻辑
                sendNextGoal();
            } else if(wait_seconds_at_waypoint_ > 0.01) { // 仅当等待时间有意义时才等待
                ROS_INFO("在路径点 [%s] 等待 %.1f 秒...", finished_waypoint_name.c_str(), wait_seconds_at_waypoint_);
                wait_timer_.setPeriod(ros::Duration(wait_seconds_at_waypoint_));
                wait_timer_.start(); // 启动一次性定时器
            } else {
                // 没有等待时间，或所有循环已完成，直接尝试发送下一个目标
                sendNextGoal();
            }
        } else {
            ROS_WARN("导航至路径点 [%s] 失败: %s", finished_waypoint_name.c_str(), state.toString().c_str());
            // 失败处理：可以选择重试当前点、跳过、或终止所有导航
            // 当前实现：跳过此失败点，继续下一个
            current_index_++;
            sendNextGoal(); // 尝试下一个目标点
        }
    }

    // 定时器回调函数
    void waitTimerCallback(const ros::TimerEvent& event) {
        boost::mutex::scoped_lock lock(mutex_); // 获取锁，因为要调用sendNextGoal
        if(!is_active_) {
            ROS_DEBUG("等待定时器触发，但导航已非激活状态。");
            return;
        }
        ROS_INFO("路径点等待结束，前往下一个目标。");
        sendNextGoal();
    }

    void goalActive() {
        // boost::mutex::scoped_lock lock(mutex_); // publishStatus 是线程安全的，ROS_DEBUG也是
        // publishStatus(true); // 移动到 sendNextGoal 发送后，或在这里保持
        ROS_DEBUG("目标已激活。");
    }

    void goalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
        // ROS_DEBUG("收到反馈：当前位置 (%.2f, %.2f)",
        //     feedback->base_position.pose.position.x,
        //     feedback->base_position.pose.position.y);
    }

    void publishStatus(bool navigating) {
        std_msgs::Bool msg;
        msg.data = navigating;
        status_pub_.publish(msg);
    }

    std::unique_ptr<MoveBaseClient> ac_;
    std::vector<Waypoint> waypoints_;

    ros::Publisher status_pub_;
    ros::ServiceServer stop_srv_;
    ros::Timer wait_timer_; // 用于在路径点等待的定时器

    boost::mutex mutex_; // 用于保护共享变量的互斥锁
    bool is_active_;       // 导航任务是否激活
    size_t current_index_; // 当前目标路径点在列表中的索引
    int loop_count_;       // 已完成的循环次数 (0-indexed)
    int total_loops_;      // 总共需要执行的循环次数 (0表示无限)
    double wait_seconds_at_waypoint_; // 在每个路径点等待的时间（秒）
    std::string frame_id_; // 目标点的坐标系ID
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); // 使用系统默认locale，通常支持UTF-8，或者用 "en_US.UTF-8"
    ros::init(argc, argv, "enhanced_navigator"); // 节点名与您launch文件中的 type 对应（如果这是可执行文件名）
                                                     // 或者与launch文件中的 name 对应（如果这是节点实例名）
                                                     // 通常，节点名在ros::init中定义，launch的name是其实例名

    EnhancedNavigation navigator;

    ros::AsyncSpinner spinner(2); // 使用2个线程处理回调，确保定时器和actionlib回调不会互相阻塞太久
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

