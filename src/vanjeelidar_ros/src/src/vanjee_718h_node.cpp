#include "vanjee_mini_lidar/vanjee_718h_node.h"

namespace vanjee_lidar
{
    Vanjee718HNode::Vanjee718HNode()
    {
        ros::NodeHandle nh("~");
        nh.getParam("lidar_ip", lidar_ip_);
        nh.getParam("lidar_port", lidar_port_);

        std::cout << "lidar_ip: " << lidar_ip_ << ",lidar_port:" << lidar_port_ << std::endl;

        lidar_protocol_ = new Vanjee718HLidarProtocol();
        dynamic_reconfigure::Server<vanjee_mini_lidar::vanjee_718h_lidarConfig>::CallbackType f;
        f = boost::bind(&Vanjee718HNode::callback, this, _1, _2);
        server_.setCallback(f);

        connect();
    }

    Vanjee718HNode::~Vanjee718HNode()
    {
        if(commumication_)
        {
            delete commumication_;
        }
        delete lidar_protocol_;
    }

    void Vanjee718HNode::callback(vanjee_mini_lidar::vanjee_718h_lidarConfig &config, uint32_t level)
    {
        lidar_protocol_->setConfig(config, level);
    }

    void Vanjee718HNode::connect()
    {
        ROS_INFO("communication_mode: TCP");
        commumication_ = new VanjeeCommonTcp(lidar_protocol_);

        while (!commumication_->connected_)
        {
            ROS_INFO("Start connecting!");
            if (commumication_->connect(lidar_ip_.c_str(), lidar_port_))
            {
                ROS_INFO("Succesfully connected. Hello vanjee_718h_lidar!");
            }
            else
            {
                ROS_INFO("Failed to connect. Waiting 5s to reconnect!");
            }
            ros::Duration(5).sleep();
        }
        lidar_protocol_->heartstate_ = false;
    }

    void Vanjee718HNode::checkConnection()
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
    ros::init(argc, argv, "vanjee_718h_lidar_01");
    
    vanjee_lidar::Vanjee718HNode *vanjee_node = new vanjee_lidar::Vanjee718HNode();
    
    while(ros::ok())
    {
        ros::Duration(2).sleep();
        vanjee_node->checkConnection();
        ros::spinOnce();
    }
    delete vanjee_node;
    return 0;
}
