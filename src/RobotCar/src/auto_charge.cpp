#include "ros/ros.h"
#include <iostream>
#include <cstring>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include "RobotCar/carinfo.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
using namespace std;

#define Tag_ID   0
#define Tag_SIZE 0.05

bool tag_detection = false;

//tag的位姿
float tag_pose_x  = 0.0;
float tag_pose_y  = 0.0;
float tag_pose_z  = 0.0;//距离
float tag_pose_A  = 0.0;

geometry_msgs::TransformStamped transformStamped_tag;
apriltag_ros::AprilTagDetectionArray detection_msg;

void tag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    // ROS_INFO("decection_cb recalled");
    if(msg->detections.empty())
    {
        ROS_INFO("No tag detected");
    }
    else
    {
        if(msg->detections[0].id[0] == Tag_ID && msg->detections[0].size[0] == Tag_SIZE)
        {
            detection_msg = *msg;
            tag_detection = true;
        }
        else
        {
            ROS_WARN("tag_id: %d and tag_size %f mismatched. Expected: tag_id: %d tag_size: %f",
            msg->detections[0].id[0], msg->detections[0].size[0], Tag_ID, Tag_SIZE);
            tag_detection = false;
        }
    }
}


int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ROS_INFO("Tag Detection Charger Started!");
    ros::init(argc, argv, "tag_detection_charger");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Subscriber apriltag_sub = nh.subscribe("/tag_detections",1,tag_callback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    double roll, pitch, yaw;
    float dif_x, dif_y;        
    while 
    
    ok()) 
    {
        if(tag_detection)//检测到二维码
        {
            try//获取二维码相对map坐标
            {
                transformStamped_tag = tfBuffer.lookupTransform("base_footprint", "tag_0", ros::Time(0));
                tf2::Quaternion quat;
                tf2::fromMsg(transformStamped_tag.transform.rotation, quat);
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换

                tag_pose_x = transformStamped_tag.transform.translation.z;
                tag_pose_y = transformStamped_tag.transform.translation.x;
                tag_pose_z = transformStamped_tag.transform.translation.y;//二维码与相机高度固定，可以不获取高度数据
                tag_pose_A = yaw;//tag的位姿
                ROS_INFO("%f,%f,%f", tag_pose_x, tag_pose_y, tag_pose_A);

            }
            catch (tf2::TransformException &ex)
            {
                tag_pose_x = 0.0;
                tag_pose_y = 0.0;
                tag_pose_z = 0.0;
                tag_pose_A = 0.0;
                ROS_WARN("Transform error: %s", ex.what());
            }
            tag_detection = false;
        }
        


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}