/**
 * @file send_goals_node.cpp
 * @brief 多点导航循环运动程序（优化版 v4）
 * * 主要优化点：
 * 1. 增强参数校验和错误处理 (特别是 waypoint 类型校验)
 * 2. 改进状态机逻辑确保运行稳定性
 * 3. 优化线程同步机制 (使用ros::Timer替代sleep)
 * 4. 完善导航状态管理
 * 5. 增加航点超时机制，超时则自动前往下一航点
 * 6. 使用 std::unique_ptr 管理 ActionClient 资源
 * 7. 增加 move_base 服务器连接重试机制
 * 8. 增加导航失败后清除代价地图的恢复机制
 * 9. 增加参数加载时的调试日志，用于定位问题
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h> // 用于 clear_costmaps 服务
#include <vector>
#include <string>
#include <memory> // 用于 std::unique_ptr
#include <boost/thread/mutex.hpp>
#include <XmlRpcValue.h>

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
        // 使用 std::make_unique 初始化 unique_ptr
        ac_(std::make_unique<MoveBaseClient>("move_base", true)),
        is_active_(false),
        current_index_(0),
        loop_count_(0),
        total_loops_(0),
        wait_seconds_at_waypoint_(3.0),
        waypoint_timeout_seconds_(0.0),
        recovery_delay_seconds_(2.0) // 新增：恢复（清除代价地图）后的等待时间
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        if(!loadParameters(private_nh)) {
            ROS_ERROR("参数加载失败。节点可能无法正常工作。");
            // 参数加载失败通常是致命的，可以考虑在此处 shutdown 或标记为非活动
            is_active_ = false; // loadParameters 会根据情况设置 is_active_，这里确保如果它返回 false，我们知道状态是非活动的
        }

        stop_srv_ = nh.advertiseService("stop_navigation", &EnhancedNavigation::stopCallback, this);
        status_pub_ = nh.advertise<std_msgs::Bool>("navigation_status", 5);

        // 创建 clear_costmaps 服务的客户端
        clear_costmaps_client_ = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
        // 可以选择等待服务就绪，但这通常不是必须的，因为move_base启动后服务才会注册

        // move_base 服务器连接重试机制
        int connection_retries = 0;
        const int max_connection_retries = 5; // 最大重试次数
        const double retry_interval_seconds = 5.0; // 重试间隔

        ROS_INFO("尝试连接move_base Action服务器...");
        while (!ac_->waitForServer(ros::Duration(5.0)) && ros::ok()) {
            connection_retries++;
            if (connection_retries > max_connection_retries) {
                ROS_ERROR("无法连接move_base Action服务器，已达到最大重试次数 (%d 次)。节点即将关闭。", max_connection_retries);
                ros::shutdown();
                return;
            }
            ROS_WARN("连接move_base Action服务器失败，将在 %.1f 秒后进行第 %d/%d 次重试...", retry_interval_seconds, connection_retries, max_connection_retries);
            ros::Duration(retry_interval_seconds).sleep(); // 此处短暂sleep用于重试间隔是可接受的，因为它在初始化阶段
            ROS_INFO("尝试重新连接move_base Action服务器 (第 %d/%d 次)...", connection_retries, max_connection_retries);
        }

        if (!ros::ok()) { // 如果在等待过程中ROS关闭了
            ROS_INFO("ROS已关闭，停止连接move_base服务器。");
            return;
        }
        ROS_INFO("move_base Action服务器已成功连接。");

        wait_timer_ = nh.createTimer(ros::Duration(0.1), &EnhancedNavigation::waitTimerCallback, this, true, false);
        goal_timeout_timer_ = nh.createTimer(ros::Duration(0.1), &EnhancedNavigation::goalTimeoutCallback, this, true, false);

        ROS_INFO("导航系统已就绪，共加载了 %zu 个路径点。", waypoints_.size()); // 使用 %zu 格式化 size_t
        if (waypoint_timeout_seconds_ > 0.0) {
            ROS_INFO("航点导航超时机制已启用，超时时间: %.1f 秒。", waypoint_timeout_seconds_);
        } else {
            ROS_INFO("航点导航超时机制已禁用。");
        }
        ROS_INFO("导航失败后的恢复延迟时间: %.1f 秒。", recovery_delay_seconds_);


        // 只有在参数加载成功且有路径点时才激活导航
        if (is_active_ && !waypoints_.empty()) {
            // is_active_ = true; // loadParameters成功时已设置为true，但如果失败了，此处保持false
            current_index_ = 0;
            loop_count_ = 0;
            ROS_INFO("自动启动导航任务。");
            sendNextGoal();
        } else if (waypoints_.empty()) {
             ROS_WARN("没有加载到任何有效的路径点，导航任务不会自动启动。");
             is_active_ = false; // 确认没有路径点时is_active_为false
             publishStatus(false);
        } else {
             // 参数加载失败，但不影响move_base连接成功，此时is_active_已在loadParameters中设为false
             ROS_WARN("参数加载失败，导航任务不会自动启动。");
             publishStatus(false);
        }
    }

private:
    bool loadParameters(ros::NodeHandle& nh) {
        is_active_ = true; // 假设参数加载会成功，除非遇到问题

        nh.param("total_loops", total_loops_, 0);
        ROS_INFO_STREAM("加载参数 'total_loops' (总循环次数): " << total_loops_);

        nh.param("wait_seconds", wait_seconds_at_waypoint_, 3.0);
        ROS_INFO_STREAM("加载参数 'wait_seconds' (航点等待秒数): " << wait_seconds_at_waypoint_);

        nh.param("waypoint_timeout_seconds", waypoint_timeout_seconds_, 0.0);
        ROS_INFO_STREAM("加载参数 'waypoint_timeout_seconds' (航点导航超时秒数): " << waypoint_timeout_seconds_);

        nh.param("recovery_delay_seconds", recovery_delay_seconds_, 2.0); // 加载新的恢复延迟参数
        ROS_INFO_STREAM("加载参数 'recovery_delay_seconds' (导航失败恢复延迟秒数): " << recovery_delay_seconds_);

        nh.param("frame_id", frame_id_, std::string("map"));
        ROS_INFO_STREAM("加载参数 'frame_id' (坐标系ID): " << frame_id_);

        XmlRpc::XmlRpcValue waypoints_list;
        if (!nh.getParam("waypoints", waypoints_list)) {
            ROS_WARN("未在参数服务器找到参数 'waypoints'。导航将不会包含任何目标点。");
            waypoints_.clear(); // 确保列表为空
            is_active_ = false; // 没有waypoints参数，标记为非活动
            return true; // 未找到参数不代表加载流程失败
        }

        if (waypoints_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("参数 'waypoints' 的格式不是列表 (Array)。导航将不会包含任何目标点。");
            waypoints_.clear(); // 确保列表为空
            is_active_ = false; // 参数格式错误，标记为非活动
            return true; // 格式错误不代表加载流程失败
        }

        waypoints_.clear();
        if (waypoints_list.size() == 0) {
            ROS_INFO("参数 'waypoints' 列表为空，没有路径点可供导航。");
            is_active_ = false; // 列表为空，标记为非活动
            return true; // 列表为空不代表加载流程失败
        }

        ROS_INFO_STREAM("在参数 'waypoints' 中配置了 " << waypoints_list.size() << " 个路径点结构。");
        for (int i = 0; i < waypoints_list.size(); ++i) {
            // --- 添加调试日志：正在处理哪个 waypoint ---
            ROS_INFO_STREAM("正在处理路径点 #" << i + 1 << "/" << waypoints_list.size() << "...");

            XmlRpc::XmlRpcValue& waypoint_param = waypoints_list[i];
            if (waypoint_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN_STREAM("路径点 #" << i + 1 << " 的格式不是一个结构体 (Struct)，类型为: " << waypoint_param.getType() << "。已跳过该路径点。");
                continue;
            }

            Waypoint waypoint;
            bool valid_waypoint = true; // 标记当前路径点是否有效

            // --- 检查并加载 x 坐标 ---
            if (waypoint_param.hasMember("x")) {
                XmlRpc::XmlRpcValue& x_value = waypoint_param["x"];
                // --- 添加调试日志：x 字段的类型 ---
                ROS_DEBUG_STREAM("路径点 #" << i + 1 << " 的 'x' 字段存在，类型为: " << x_value.getType());
                if (x_value.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                    x_value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
                    x_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) // XmlRpc can cast bool/int to double
                {
                    waypoint.x = static_cast<double>(x_value);
                } else {
                    ROS_WARN_STREAM("路径点 #" << i + 1 << " 的 'x' 字段类型不正确 (期待数字类型，实际类型为: " << x_value.getType() << ")。已跳过该路径点。");
                    valid_waypoint = false;
                }
            } else {
                ROS_WARN_STREAM("路径点 #" << i + 1 << " 缺少必要的 'x' 坐标字段。已跳过该路径点。");
                valid_waypoint = false;
            }

            if (!valid_waypoint) continue; // 如果x无效，跳过整个路径点

            // --- 检查并加载 y 坐标 ---
            if (waypoint_param.hasMember("y")) {
                 XmlRpc::XmlRpcValue& y_value = waypoint_param["y"];
                 // --- 添加调试日志：y 字段的类型 ---
                 ROS_DEBUG_STREAM("路径点 #" << i + 1 << " 的 'y' 字段存在，类型为: " << y_value.getType());
                 if (y_value.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                     y_value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
                     y_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                 {
                     waypoint.y = static_cast<double>(y_value);
                 } else {
                     ROS_WARN_STREAM("路径点 #" << i + 1 << " 的 'y' 字段类型不正确 (期待数字类型，实际类型为: " << y_value.getType() << ")。已跳过该路径点。");
                     valid_waypoint = false;
                 }
            } else {
                 ROS_WARN_STREAM("路径点 #" << i + 1 << " 缺少必要的 'y' 坐标字段。已跳过该路径点。");
                 valid_waypoint = false;
            }

            if (!valid_waypoint) continue; // 如果y无效，跳过整个路径点

            // --- 检查并加载 theta 朝向 (可选字段) ---
            if (waypoint_param.hasMember("theta")) {
                 XmlRpc::XmlRpcValue& theta_value = waypoint_param["theta"];
                 // --- 添加调试日志：theta 字段的类型 ---
                 ROS_DEBUG_STREAM("路径点 #" << i + 1 << " 的 'theta' 字段存在，类型为: " << theta_value.getType());
                 if (theta_value.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                     theta_value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
                     theta_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                 {
                     waypoint.theta = static_cast<double>(theta_value);
                 } else {
                     ROS_WARN_STREAM("路径点 #" << i + 1 << " 的 'theta' 字段类型不正确 (期待数字类型，实际类型为: " << theta_value.getType() << ")。使用默认值 0.0 rad。");
                     waypoint.theta = 0.0; // 类型错误，使用默认值但不跳过
                 }
            } else {
                 waypoint.theta = 0.0;
                 ROS_DEBUG_STREAM("路径点 #" << i + 1 << " 缺少 'theta' (朝向角)字段，使用默认值 0.0 rad。");
            }

            // --- 检查并加载 name 名称 (可选字段) ---
            if (waypoint_param.hasMember("name")) {
                 XmlRpc::XmlRpcValue& name_value = waypoint_param["name"];
                 // --- 添加调试日志：name 字段的类型 ---
                 ROS_DEBUG_STREAM("路径点 #" << i + 1 << " 的 'name' 字段存在，类型为: " << name_value.getType());
                 if (name_value.getType() == XmlRpc::XmlRpcValue::TypeString)
                 {
                    waypoint.name = static_cast<std::string>(name_value);
                 } else {
                    ROS_WARN_STREAM("路径点 #" << i + 1 << " 的 'name' 字段类型不正确 (期待字符串类型，实际类型为: " << name_value.getType() << ")。使用默认名称。");
                    waypoint.name = "point_" + std::to_string(i + 1); // 类型错误，使用默认名称
                 }
            } else {
                 waypoint.name = "point_" + std::to_string(i + 1);
                 ROS_DEBUG_STREAM("路径点 #" << i + 1 << " 缺少 'name' (名称)字段，使用默认值 '" << waypoint.name << "'。");
            }

            // 如果所有必要的字段都有效且类型正确，则添加到列表
            if (valid_waypoint) {
                waypoints_.push_back(waypoint);
                // --- 添加调试日志：成功处理一个路径点 ---
                ROS_INFO_STREAM("成功加载路径点 #" << i + 1 << ".");
            } else {
                 ROS_WARN_STREAM("路径点 #" << i + 1 << " 因校验失败被跳过。");
            }

        } // for loop end

        if (waypoints_.empty()) {
            ROS_WARN("经过参数解析和校验后，有效的路径点列表为空。导航任务不会启动。");
            is_active_ = false; // 没有有效路径点，标记为非活动
            return true; // 参数加载过程本身完成，只是结果为空
        }

        ROS_INFO("成功加载了 %zu 个有效路径点。", waypoints_.size()); // 使用 %zu 格式化 size_t
        is_active_ = true; // 成功加载了有效路径点，标记为活动
        return true;
    }

    bool stopCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        boost::mutex::scoped_lock lock(mutex_);

        if(is_active_) {
            ROS_INFO("接收到外部指令：停止当前导航任务。");
            ac_->cancelAllGoals();
            wait_timer_.stop();
            goal_timeout_timer_.stop();
            is_active_ = false;
            publishStatus(false);
            ROS_INFO("导航任务已成功终止。");
        } else {
            ROS_INFO("导航任务当前未激活，无需执行停止操作。");
        }
        return true;
    }

    void sendNextGoal() {
        // 在发送目标前再次确认是否激活且有路径点
        if(!ros::ok() || !ac_ || !is_active_ || waypoints_.empty()) {
            ROS_DEBUG("sendNextGoal：条件不满足 (ros::ok: %s, ac_: %s, is_active_: %s, waypoints_.empty(): %s)，不发送新目标。",
                      ros::ok() ? "true" : "false", ac_ ? "true" : "false", is_active_ ? "true" : "false", waypoints_.empty() ? "true" : "false");
            goal_timeout_timer_.stop();
            wait_timer_.stop();
            publishStatus(false);
            // 如果是因为没有路径点而停止，需要在这里返回，否则可能进入循环处理
            if (waypoints_.empty() && !is_active_) return;
            // 如果是因为is_active_=false而停止，也返回
            if (!is_active_) return;
        }


        if(current_index_ >= waypoints_.size()) {
            if(total_loops_ > 0) {
                loop_count_++;
                if(loop_count_ >= total_loops_) {
                    ROS_INFO("已完成预设的 %d 次导航循环。导航任务结束。", total_loops_);
                    is_active_ = false;
                    wait_timer_.stop();
                    goal_timeout_timer_.stop();
                    publishStatus(false);
                    return;
                }
            }
            current_index_ = 0;
            ROS_INFO("开始第 %d 次导航循环。", loop_count_ + 1);
        }

        // 确保 current_index_ 在有效范围内 (理论上到这里不会越界，但以防万一)
        if (current_index_ >= waypoints_.size()) {
             ROS_ERROR("内部错误：current_index_ (%zu) 超出 waypoint 范围 (%zu)。停止导航。", current_index_, waypoints_.size());
             is_active_ = false; // 索引越界，停止导航
             publishStatus(false);
             return;
        }
        const auto& wp = waypoints_[current_index_];
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = frame_id_;

        goal.target_pose.pose.position.x = wp.x;
        goal.target_pose.pose.position.y = wp.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, wp.theta);
        goal.target_pose.pose.orientation = tf2::toMsg(q);

        ROS_INFO("第 %d/%s 次循环，前往路径点 [%s] (序号 %zu/%zu): 坐标 (%.2f, %.2f), 朝向 %.2f rad",
                 loop_count_ + 1,
                 (total_loops_ == 0 ? "无限" : std::to_string(total_loops_).c_str()),
                 wp.name.c_str(),
                 current_index_ + 1,
                 waypoints_.size(),
                 wp.x, wp.y, wp.theta);

        ac_->sendGoal(goal,
                      boost::bind(&EnhancedNavigation::goalDone, this, _1, _2),
                      boost::bind(&EnhancedNavigation::goalActive, this),
                      boost::bind(&EnhancedNavigation::goalFeedback, this, _1));
        publishStatus(true);

        // 启动航点超时定时器
        if (waypoint_timeout_seconds_ > 0.0) {
            goal_timeout_timer_.stop(); // 先停止，确保从0开始计时
            goal_timeout_timer_.setPeriod(ros::Duration(waypoint_timeout_seconds_));
            goal_timeout_timer_.start();
            ROS_DEBUG("路径点 [%s] 的导航超时定时器已启动 (%.1f 秒)。", wp.name.c_str(), waypoint_timeout_seconds_);
        }
    }

    void goalDone(const actionlib::SimpleClientGoalState& state,
                  const move_base_msgs::MoveBaseResultConstPtr& result) {
        boost::mutex::scoped_lock lock(mutex_);
        if(!ros::ok() || !ac_) return; // 增加对ac_有效性的检查

        goal_timeout_timer_.stop(); // 目标完成，无论成功失败，停止超时定时器

        if(!is_active_) { // 如果任务已经被停止，直接返回
            ROS_DEBUG("goalDone: 导航任务未激活 (is_active_ 为 false)，目标完成回调被忽略。");
            return;
        }

        // 确保 completed_waypoint_index 在有效范围内
        // 注意：goalDone触发时，current_index_还是当前目标的索引
        size_t completed_waypoint_index = current_index_;
        std::string finished_waypoint_name = "未知路径点";
        if (completed_waypoint_index < waypoints_.size()) {
             finished_waypoint_name = waypoints_[completed_waypoint_index].name;
        } else {
             ROS_WARN("goalDone: completed_waypoint_index (%zu) 超出 waypoint 范围 (%zu)。", completed_waypoint_index, waypoints_.size());
             // 如果索引越界，可能参数加载有问题或者循环逻辑有问题，最好停止
             is_active_ = false;
             publishStatus(false);
             return; // 停止并返回
        }


        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            // 导航成功
            ROS_INFO("成功到达路径点 [%s] (序号 %zu)。", finished_waypoint_name.c_str(), completed_waypoint_index + 1);
            current_index_++; // 准备前往下一个路径点

            bool is_last_waypoint_of_last_loop = false;
            // 检查是否到达了最后一个循环的最后一个路径点
            if (current_index_ >= waypoints_.size() && total_loops_ > 0 && (loop_count_ + 1) >= total_loops_) {
                is_last_waypoint_of_last_loop = true;
            }

            if (is_last_waypoint_of_last_loop) {
                ROS_INFO("已到达最后一个指定循环的最后一个路径点，准备结束导航任务。");
                // sendNextGoal() 会检测到循环结束并停止 is_active_
                sendNextGoal();
            } else if(wait_seconds_at_waypoint_ > 0.01) {
                // 如果设置了等待时间，启动等待定时器
                ROS_INFO("在路径点 [%s] 等待 %.1f 秒...", finished_waypoint_name.c_str(), wait_seconds_at_waypoint_);
                wait_timer_.stop(); // 先停止，确保从0开始计时
                wait_timer_.setPeriod(ros::Duration(wait_seconds_at_waypoint_));
                wait_timer_.start();
            } else {
                // 没有等待时间，直接前往下一个目标
                sendNextGoal();
            }
        } else {
            // 导航失败或被取消
            ROS_WARN("导航至路径点 [%s] (序号 %zu) 失败或被取消，状态: %s。",
                         finished_waypoint_name.c_str(),
                         completed_waypoint_index + 1,
                         state.toString().c_str());

            if (state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::REJECTED) {
                // 特殊处理导航失败 (ABORTED 或 REJECTED)，尝试恢复
                ROS_WARN("检测到导航失败（状态: %s）。尝试清除代价地图并前往下一个路径点...", state.toString().c_str());

                std_srvs::Empty srv;
                if (clear_costmaps_client_.call(srv)) {
                    ROS_INFO("成功调用 clear_costmaps 服务。");
                } else {
                    // 调用失败可能是move_base挂了或者服务不存在，需要额外处理？这里先只打日志
                    ROS_WARN("调用 clear_costmaps 服务失败。");
                }

                // 在清除代价地图后短暂等待，给系统恢复时间
                if (recovery_delay_seconds_ > 0.01) {
                    ROS_INFO("等待 %.1f 秒后前往下一个路径点...", recovery_delay_seconds_);
                    ros::Duration(recovery_delay_seconds_).sleep(); // 在回调中短时间sleep是可接受的
                }

            } else {
                // 其他非成功状态 (如 PREEMPTED, RECALLED, LOST) 不执行清除代价地图
                // PREEMPTED 通常是用户取消或超时取消，RECALLED 是目标被新的目标取代，LOST 是 ActionServer 挂了
                // 在这些情况下清除代价地图可能不是必要的，直接去下一个点更合理
                ROS_INFO("导航状态为 %s，直接前往下一个路径点。", state.toString().c_str());
            }

            current_index_++; // 总是尝试前往下一个路径点（即使失败或被取消）
            sendNextGoal();
        }
    }

    void waitTimerCallback(const ros::TimerEvent& event) {
        boost::mutex::scoped_lock lock(mutex_);
        if(!ros::ok()) return;

        if(!is_active_) {
            ROS_DEBUG("waitTimerCallback: 等待定时器触发，但导航任务未激活。");
            return;
        }
        ROS_INFO("路径点等待时间结束，准备前往下一个目标。");
        sendNextGoal();
    }

    void goalTimeoutCallback(const ros::TimerEvent& event) {
        boost::mutex::scoped_lock lock(mutex_);
        if(!ros::ok() || !ac_) return; // 增加对ac_有效性的检查

        if(!is_active_) {
            ROS_DEBUG("goalTimeoutCallback: 航点导航超时定时器触发，但导航任务未激活。");
            goal_timeout_timer_.stop(); // 如果任务停止了，确保定时器也停
            return;
        }

        // 在超时回调中，我们只关心目标是否仍在进行中
        actionlib::SimpleClientGoalState current_goal_state = ac_->getState();
        // 如果目标已经完成（成功或失败），超时回调就不需要做任何事了
        if (current_goal_state.isDone()) {
            ROS_DEBUG("goalTimeoutCallback: 航点导航超时定时器触发，但目标此前已完成 (%s)。不执行任何操作。", current_goal_state.toString().c_str());
            goal_timeout_timer_.stop(); // 停止定时器
            return;
        }

        // 如果目标仍在进行中且超时了
        // 使用 current_index_ 来确定是哪个点超时，但要注意这个索引在 goalDone 里才会++
        size_t timed_out_waypoint_index = current_index_;
        std::string current_waypoint_name = "未知路径点 (超时处理时)";
        if(timed_out_waypoint_index < waypoints_.size()){
            current_waypoint_name = waypoints_[timed_out_waypoint_index].name;
        } else {
             ROS_WARN("goalTimeoutCallback: timed_out_waypoint_index (%zu) 超出 waypoint 范围 (%zu)。可能在处理最后一个点时超时。", timed_out_waypoint_index, waypoints_.size());
             // 如果索引越界，可能参数加载有问题或者循环逻辑有问题，最好停止
             is_active_ = false;
             publishStatus(false);
             ac_->cancelGoal(); // 尝试取消当前可能的活动目标
             return; // 停止并返回
        }


        ROS_WARN("导航至路径点 [%s] (序号 %zu/%zu) 超时 (%.1f 秒)！正在取消当前目标...",
                 current_waypoint_name.c_str(), timed_out_waypoint_index + 1, waypoints_.size(), waypoint_timeout_seconds_);

        ac_->cancelGoal(); // 取消目标，这将触发 goalDone 回调，然后在 goalDone 中处理失败并前往下一个点
        // 注意：取消目标后，goalDone 会以 PREEMPTED 状态触发，它会直接跳到下一个点（不清除代价地图）。
        // 如果你希望超时后也清除代价地图，可以在这里（goalTimeoutCallback）调用 clear_costmaps 服务，
        // 或者在 goalDone 中专门处理 PREEMPTED 状态，但清除代价地图通常只针对 ABORTED/REJECTED 更合理。
        // 当前实现是在 goalDone 中针对 ABORTED/REJECTED 清除，对 PREEMPTED 则跳过，这是较常见的逻辑。
    }


    void goalActive() {
        ROS_DEBUG("Action目标已激活 (goalActive callback)。");
        // 目标激活时可以做一些事情，例如记录时间戳用于计算超时（如果不用ros::Timer的话）
    }

    void goalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
        // 导航反馈，可以用来显示当前位置或进度
        // ROS_DEBUG_THROTTLE(1.0, "收到导航反馈：当前机器人位置 (%.2f, %.2f)",
        //                   feedback->base_position.pose.position.x,
        //                   feedback->base_position.pose.position.y);
    }

    void publishStatus(bool navigating) {
        std_msgs::Bool msg;
        msg.data = navigating;
        status_pub_.publish(msg);
    }

    // 将 ac_ 声明为 std::unique_ptr
    std::unique_ptr<MoveBaseClient> ac_;
    std::vector<Waypoint> waypoints_;

    ros::Publisher status_pub_;
    ros::ServiceServer stop_srv_;
    ros::Timer wait_timer_;
    ros::Timer goal_timeout_timer_;
    ros::ServiceClient clear_costmaps_client_; // 清除代价地图服务的客户端

    boost::mutex mutex_;
    bool is_active_;
    size_t current_index_;
    int loop_count_;
    int total_loops_;
    double wait_seconds_at_waypoint_;
    double waypoint_timeout_seconds_;
    double recovery_delay_seconds_; // 新增：恢复（清除代价地图）后的等待时间
    std::string frame_id_;
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "enhanced_navigator");

    // 设置日志级别，以便看到 DEBUG 级别的输出
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
     // 也可以为特定的命名空间设置日志级别，例如 "ros.enhanced_navigator"
    if( ros::console::set_logger_level("ros.enhanced_navigator", ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }


    EnhancedNavigation navigator;

    // 构造函数中如果move_base连接失败可能已经shutdown，需要检查
    if (!ros::ok()) {
        ROS_ERROR("节点因ROS未就绪或初始化失败而退出。");
        return 1;
    }

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    ROS_INFO("导航节点已正常关闭。");

    return 0;
}