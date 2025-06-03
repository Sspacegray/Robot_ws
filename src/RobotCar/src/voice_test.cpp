#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <RobotCar/robotinfo.h>
#include <RobotCar/web2robot.h>
#include <RobotCar/voice2robot.h>

using namespace std;

// 控制模式枚举
enum ControlMode {
    WEB_CONTROL = 0,
    VOICE_CONTROL = 1
};

// 机器人状态枚举 - 与robot_path_control.cpp保持一致
enum RobotState {
    STOP        = 0,
    RUNNING     = 1,
    A_OPEN_DOOR = 2,
    B_OPEN_DOOR = 3,
    FINISH      = 4,
    WAITING     = 5,
    BACK_TO_0   = 6,
    CHARGE      = 7,
    FAILED      = 8,
    START       = 9,
};

// 全局变量

ros::ServiceClient voice2robot_client;
RobotCar::voice2robot srv;
ros::Publisher voice_mode_pub;     // 发布语音模式
std_msgs::Int8 voice_mode;         // 语音模式消息
// RobotCar::carinfo car_info_msg;    // 机器人控制消息
ros::Publisher control_mode_pub;   // 发布控制模式
std_msgs::Int8 control_mode_msg;   // 控制模式消息
ros::Publisher awake_flag_pub;     // 唤醒标志发布者
std_msgs::Int8 awake_flag_msg;     // 唤醒标志消息

ControlMode current_control_mode = WEB_CONTROL;  // 当前控制模式，默认为Web控制
int current_room = -1;                           // 当前目标房间号
bool waiting_for_confirmation = false;           // 是否等待确认
bool is_navigating = false;                      // 是否正在导航
ros::Time last_activity_time;                    // 上次活动时间，用于超时检测
u_int32_t current_robot_state;                   //机器人当前状态
u_int32_t last_robot_state;                      //机器人上一次状态

// 语音提示结构体
struct VoicePrompt {
    string confirm_message;    // 确认消息
    string going_message;      // 前往消息
    string confirm_sound;      // 确认语音文件
    string going_sound;        // 执行语音文件
    string arrived_sound;      // 到达语音文件
};

// 预定义的语音提示 - 包含充电位(0)
vector<VoicePrompt> voice_prompts = {
    {"确认去往充电位吗", "小达正在前往充电位", "确认房间/0.mp3", "正在前进/0.mp3", "已到达/0.mp3"},
    {"确认去往房间1吗", "小达正在前往房间1", "确认房间/1.mp3", "正在前进/1.mp3", "已到达/1.mp3"},
    {"确认去往房间2吗", "小达正在前往房间2", "确认房间/2.mp3", "正在前进/2.mp3", "已到达/2.mp3"},
    {"确认去往房间3吗", "小达正在前往房间3", "确认房间/3.mp3", "正在前进/3.mp3", "已到达/3.mp3"},
    {"确认去往房间4吗", "小达正在前往房间4", "确认房间/4.mp3", "正在前进/4.mp3", "已到达/4.mp3"},
    {"确认去往房间5吗", "小达正在前往房间5", "确认房间/5.mp3", "正在前进/5.mp3", "已到达/5.mp3"},
    {"确认去往房间6吗", "小达正在前往房间6", "确认房间/6.mp3", "正在前进/6.mp3", "已到达/6.mp3"},
    {"确认去往房间7吗", "小达正在前往房间7", "确认房间/7.mp3", "正在前进/7.mp3", "已到达/7.mp3"},
    {"确认去往房间8吗", "小达正在前往房间8", "确认房间/8.mp3", "正在前进/8.mp3", "已到达/8.mp3"},
    {"确认去往房间9吗", "小达正在前往房间9", "确认房间/9.mp3", "正在前进/9.mp3", "已到达/9.mp3"},
    {"确认去往房间10吗", "小达正在前往房间10", "确认房间/10.mp3", "正在前进/10.mp3", "已到达/10.mp3"},
    {"确认去往房间11吗", "小达正在前往房间11", "确认房间/11.mp3", "正在前进/11.mp3", "已到达/11.mp3"}
};

// 机器人状态提示
vector<string> status_prompts = {
  
    "小达已停止前进"// STOP        = 0,
    "小达正在前往目标"// RUNNING     = 1,
    "小达已到达开门点A"// A_OPEN_DOOR = 2,
    "小达已到达开门点B"// B_OPEN_DOOR = 3,
    "小达已安全到达目的地"// FINISH      = 4,
    "小达正在等待"// WAITTING    = 5,
    "小达正在返回充电位"// BACK_TO_0   = 6,
    "小达正在充电"// CHARGE      = 7,
    "小达出现了一点小问题"// FAILED      = 8,
    "小达已启动"// START = 9,
};

// 音频播放函数
void play_audio(const string& file_path) {
    string cmd = "play /home/hy/Robot_ws/src/mp3" + file_path;
    system(cmd.c_str());
}

// 解析并处理房间号命令
int parse_room_number(const string& command) {
    // 添加更多的表达方式支持
    if (command.find("充电") != string::npos || 
        command.find("回充") != string::npos || 
        command.find("回家") != string::npos || 
        command.find("充电站") != string::npos) {
        return 0;
    }
    
    // 多样化的表达方式
    size_t pos = string::npos;  //pos是字符串中第一个字符的位置
    const vector<string> room_prefixes = {"前往房间", "去房间", "去往房间", "到房间", "走到房间", "移动到房间"};
    
    for (const auto& prefix : room_prefixes) {
        if ((pos = command.find(prefix)) != string::npos) {
            pos += prefix.length();
            break;
        }
    }
    
    // 如果没找到前缀，尝试直接查找"房间"
    if (pos == string::npos) {
        pos = command.find("房间");
        if (pos != string::npos) {
            pos += 2; // "房间"的长度
        }
    }
    
    // 提取并转换房间号
    if (pos != string::npos && pos < command.length()) {
        string room_str = command.substr(pos);
        // 支持中文数字转换
        map<string, int> cn_num = {{"零", 0}, {"一", 1},  {"四", 4},
                                  {"五", 5}, {"六", 6}, {"七", 7}, {"八", 8}, {"九", 9},
                                  {"十", 10}, {"十一", 11}};
                                  
        for (const auto& pair : cn_num) {
            if (room_str.find(pair.first) == 0) {
                return pair.second;
            }
        }
        
        // 尝试解析数字
        try {
            return stoi(room_str);
        } catch (...) {
            return -1;
        }
    }
    
    return -1;
}

// 发送导航命令
void send_navigation_command(int room_number) {
    // 设置导航消息
    srv.request.room_point = room_number;
    voice2robot_client.call(srv);
    ROS_INFO("send_navigation_command: %d", room_number);
    //通过voice2robot服务来发布房间号**********************************************
}


// 休眠语音控制
void sleep_voice_control() {
    current_control_mode = WEB_CONTROL;
    control_mode_msg.data = WEB_CONTROL;
    control_mode_pub.publish(control_mode_msg);
    
    // 发布休眠标志
    awake_flag_msg.data = 0;
    awake_flag_pub.publish(awake_flag_msg);
    
    ROS_INFO("语音控制休眠");
    play_audio("others/已休眠.mp3");
}

// 语音命令回调函数
void voice_words_callback(const std_msgs::String& msg) {
    string str = msg.data;
    
    // 添加调试信息
    ROS_INFO("收到语音命令: %s", str.c_str());
    ROS_INFO("当前控制模式: %s", (current_control_mode == VOICE_CONTROL ? "语音控制" : "Web控制"));
    
    // 处理唤醒命令
    if (str == "小车唤醒" || str == "小达小达") {
        if (current_control_mode == WEB_CONTROL) {
            current_control_mode = VOICE_CONTROL;
            control_mode_msg.data = VOICE_CONTROL;
            control_mode_pub.publish(control_mode_msg);
            
            // 发布唤醒标志
            awake_flag_msg.data = 1;
            awake_flag_pub.publish(awake_flag_msg);
            
            cout << "切换到语音控制模式" << endl;
            play_audio("others/已唤醒.mp3");
        }
        return;
    }

    // 添加调试信息：解析房间号
    int room = parse_room_number(str);
    ROS_INFO("解析到的房间号: %d", room);
    
    // 仅在语音控制模式下处理其他命令
    if (current_control_mode == VOICE_CONTROL) {
        // 更新活动时间
        last_activity_time = ros::Time::now();
        
        // 处理休眠命令
        if (str == "休眠" || str == "小达休眠") {
            sleep_voice_control();
            return;
        }
        // 处理确认命令
        else if (str == "确认" && waiting_for_confirmation) {
            waiting_for_confirmation = false;
            is_navigating = true;
            
            // 发送导航命令
            send_navigation_command(current_room);
            ROS_INFO("语音控制命令回调函数:发送导航命令: %d", current_room);
            // 播放确认音频
            cout << voice_prompts[current_room].going_message << endl;
            play_audio(voice_prompts[current_room].going_sound);
            
            return;
        }
        // 直接处理"前往充电位"等无需确认的命令
        else if (str == "前往充电位" || str == "返回充电位" || str == "回去充电" || str == "充电") {
            current_room = 0; // 充电位
            is_navigating = true;
            
            send_navigation_command(current_room);
            cout << voice_prompts[current_room].going_message << endl;
            play_audio(voice_prompts[current_room].going_sound);
            // play_audio("~/Robotcar/src/mp3/机器人状态/1.mp3"); // START = 1
            return;
        }
            // 处理无法识别的命令
        else if (str == "没听清，请你再说一遍" || str == "识别失败") {
            play_audio("others/say_again.mp3");
            return;
        }
                // 如果正在导航中，不处理新的房间命令
        if (is_navigating) {
            cout << "正在前往目标，请完成任务后再试" << endl;
            play_audio("机器人状态/任务进行中.mp3");
            return;
        }
        // 处理房间命令
        
        if (room >= 0 && room < voice_prompts.size()) {
            current_room = room;
            
            // 如果是充电位，直接执行无需确认
            if (room == 0) {
                is_navigating = true;
                send_navigation_command(current_room);
                cout << voice_prompts[current_room].going_message << endl;
                play_audio("确认房间" + voice_prompts[current_room].going_sound);
                // play_audio("~/Robotcar/src/mp3/机器人状态/1.mp3"); // START = 1
                return;
            }
            else
            {
                waiting_for_confirmation = true;//等待确认标志位
                // 播放确认提示
                voice_mode.data = room;
                voice_mode_pub.publish(voice_mode);
                cout << voice_prompts[room].confirm_message << endl;
                play_audio(voice_prompts[room].confirm_sound);//播报”确认是房间**吗“
            }
            return;
        }
        else
        {
                    // 如果命令无效，播放提示
            cout << "房间点不存在，请重说" << endl;
            play_audio("others/error.mp3");
        }
    }
}

void robot_state_play_audio(void)
{
    
        // 根据状态播放对应提示音
    if (current_robot_state >= 0 && current_robot_state != last_robot_state) 
    {
        ROS_INFO("robot_state_play_audio: %d", current_robot_state);
        play_audio("机器人状态/" + to_string(current_robot_state) + ".mp3");
        
        // 处理特殊状态
        if (current_robot_state == FINISH && last_robot_state == RUNNING) // Running to finish
        {
            is_navigating = false;
            
            // 如果当前房间有效，播放到达提示
            if (current_room >= 0 && current_room < voice_prompts.size()) 
            {
                play_audio( voice_prompts[current_room].arrived_sound);
                // 如果是充电位，等待10秒后播放充电false
                // 到达目标后自动休眠语音控制
                if (current_control_mode == VOICE_CONTROL) 
                {
                ros::Duration(3.0).sleep(); // 等待3秒后休眠
                sleep_voice_control();
                }
            } 
        }
        else if (current_robot_state == FAILED && last_robot_state != current_robot_state)  //
        {
            is_navigating = false;
            play_audio("机器人状态/failed.mp3"); 
        } 
        else if (current_robot_state == BACK_TO_0 && last_robot_state == WAITING) 
        {
            // 返回充电位状态
            is_navigating = true;
             // 播放出发提示
            play_audio("机器人状态/back.mp3"); 
        }
        else if(current_robot_state == RUNNING && current_robot_state != last_robot_state)
        {
             // 播放出发提示
            is_navigating = true;
            play_audio("机器人状态/go.mp3"); 
        }
        else if(current_robot_state == START && last_robot_state != current_robot_state)
        {
            is_navigating = false;
        }
        else if(current_robot_state == WAITING && last_robot_state != current_robot_state)
        {
            is_navigating = false;
        }
    }
    last_robot_state = current_robot_state;

}

// 机器人状态回调函数
void robot_state_callback(const RobotCar::robotinfo& msg) 
{
    current_robot_state = msg.robotstate;//获取机器人当前状态
    // ROS_INFO("robot_state_callback: %d", current_robot_state);

}

// Web控制命令回调函数//重写
bool web_cmd_callback(RobotCar::web2robot::Request& req,RobotCar::web2robot::Response& res) {
    // 只有在Web控制模式下处理Web命令
    if (current_control_mode == WEB_CONTROL) {
        // 处理Web发来的导航命
        int room = req.room_point;
        if (room >= 0 && room < voice_prompts.size()) {
            current_room = room;
            send_navigation_command(current_room);//下发房间号
            ROS_INFO("Web控制命令回调函数:下发房间号: %d", current_room);
            is_navigating = true;//正在导航
            res.goal_point = current_room;
                // 更新当前发送的控制消息
        }
        else
        {
            res.goal_point = -1;
        }
    }
    else
    {
        res.goal_point = -1;
    }
    return 0;
}

// 唤醒标志回调函数
void awake_flag_callback(const std_msgs::Int8& msg) {
    if (msg.data == 1) {
        // 唤醒状态，切换到语音控制模式
        current_control_mode = VOICE_CONTROL;
        control_mode_msg.data = VOICE_CONTROL;
        control_mode_pub.publish(control_mode_msg);
        cout << "切换到语音控制模式" << endl;
    } else {
        // 休眠状态，切换回Web控制模式
        current_control_mode = WEB_CONTROL;
        control_mode_msg.data = WEB_CONTROL;
        control_mode_pub.publish(control_mode_msg);
        cout << "切换到Web控制模式" << endl;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "voice_test");
    ros::NodeHandle n;

    // 创建发布者
    //
    voice_mode_pub = n.advertise<std_msgs::Int8>("/voice_mode", 10);
    
    voice2robot_client = n.serviceClient<RobotCar::voice2robot>("voice2robot_server");
    ros::ServiceServer web2robot_srv = n.advertiseService("WEB2ROBOT_server",web_cmd_callback);
    control_mode_pub = n.advertise<std_msgs::Int8>("/control_mode", 10);
    awake_flag_pub = n.advertise<std_msgs::Int8>("/awake_flag", 10);

    // 创建订阅者
    // 订阅voice2robot_server,用于发布房间号服务
    // 订阅robotinfo话题，用于获取机器人当前状态
    ros::Subscriber voice_words_sub = n.subscribe("voice_words", 10, voice_words_callback);
    ros::Subscriber robot_state_sub = n.subscribe("/robot_info", 10, robot_state_callback);
    ros::Subscriber awake_flag_sub = n.subscribe("/awake_flag", 10, awake_flag_callback);

    // 初始化其他状态
    control_mode_msg.data = WEB_CONTROL;
    control_mode_pub.publish(control_mode_msg);
    awake_flag_msg.data = 0;
    awake_flag_pub.publish(awake_flag_msg);
    last_activity_time = ros::Time::now();
    current_robot_state = 0;
    last_robot_state = 0;
    srv.request.room_point = 0;
    

    // 设置循环频率
    ros::Rate loop_rate(1); // 10Hz

    ROS_INFO("语音控制系统已启动，等待唤醒...");

    // 主循环
    while (ros::ok()) {
        
        robot_state_play_audio();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
