#include "../include/vanjee_716mini_node.h"

namespace vanjee_lidar
{
    Vanjee716MiniNode::Vanjee716MiniNode()
    {
        ros::NodeHandle nh("~");
        nh.getParam("lidar_ip", lidar_ip_);
        nh.getParam("lidar_port", lidar_port_);
        nh.getParam("host_ip", host_ip_);
        nh.getParam("host_port", host_port_);
        nh.getParam("communication_mode", communication_mode_);

        std::cout << "host_ip: " << host_ip_ << ",host_port:" << host_port_ << std::endl;
        std::cout << "lidar_ip: " << lidar_ip_ << ",lidar_port:" << lidar_port_ << std::endl;

        lidar_protocol_ = new Vanjee716MiniLidarProtocol();
        dynamic_reconfigure::Server<RobotCar::vanjee_716mini_lidarConfig>::CallbackType f;
        f = boost::bind(&Vanjee716MiniNode::callback, this, _1, _2);
        server_.setCallback(f);

        connect();
    }

    Vanjee716MiniNode::~Vanjee716MiniNode()
    {
        if(commumication_)
        {
            delete commumication_;
        }
        delete lidar_protocol_;
    }

    void Vanjee716MiniNode::callback(RobotCar::vanjee_716mini_lidarConfig &config, uint32_t level)
    {
        lidar_protocol_->setConfig(config, level);
    }

    void Vanjee716MiniNode::connect()
    {
        if (communication_mode_ == 0) // tcp
        {
            ROS_INFO("communication_mode: TCP");
            commumication_ = new VanjeeCommonTcp(lidar_protocol_);
        }
        else if (communication_mode_ == 1) // udp
        {
            ROS_INFO("communication_mode: UDP");
            commumication_ = new VanjeeCommonUdp(host_ip_, host_port_, lidar_protocol_);
        }
        while (!commumication_->connected_)
        {
            ROS_INFO("Start connecting!");
            if (commumication_->connect(lidar_ip_.c_str(), lidar_port_))
            {
                ROS_INFO("Succesfully connected. Hello vanjee_716mini_lidar!");
            }
            else
            {
                ROS_INFO("Failed to connect. Waiting 5s to reconnect!");
            }
            ros::Duration(5).sleep();
        }
        lidar_protocol_->heartstate_ = false;
    }

    void Vanjee716MiniNode::checkConnection()
    {
        if (commumication_->connected_)
        {
            if (lidar_protocol_->heartstate_)
            {
                lidar_protocol_->heartstate_ = false;
            }
            else
            {
                commumication_->connected_ = false;
            }
        }
        else
        {
            if (!commumication_->reconnecting_)
            {
                boost::thread t(boost::bind(&VanjeeCommonAbstract::reconnect, commumication_));
            }
        }
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "vanjee_716mini_lidar_01");
    
    vanjee_lidar::Vanjee716MiniNode *vanjee_node = new vanjee_lidar::Vanjee716MiniNode();
    
    while(ros::ok())
    {
        ros::Duration(2).sleep();
        vanjee_node->checkConnection();
        ros::spinOnce();
    }
    delete vanjee_node;
    return 0;
}
