#ifndef VANJEE_ABSTRACT_PROTOCOL_H
#define VANJEE_ABSTRACT_PROTOCOL_H
#include <iostream>
#include <stdio.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>


namespace vanjee_lidar
{
#define MAX_LENGTH_DATA_PROCESS 200000
    typedef struct TagDataCache
    {
        unsigned char cache_data[MAX_LENGTH_DATA_PROCESS];
        unsigned int cache_in;
        unsigned int cache_out;
    } DataCache;

    class VanjeeAbstractProtocol
    {
    public:
        VanjeeAbstractProtocol();
        virtual ~VanjeeAbstractProtocol(){};
        bool dataProcess(unsigned char *data, const int reclen);
        virtual bool protocl(unsigned char *data, const int len) = 0;
        bool OnRecvProcess(unsigned char *data, int len);
        bool checkXor(unsigned char *recvbuf, int recvlen);
        // void sendScan(const char *data,const int len);
        ros::NodeHandle nh_;
        ros::Publisher marker_pub_;
        sensor_msgs::LaserScan scan_;

    private:
        void moveData(DataCache &sdata);
        DataCache data_;
    };
}
#endif // VANJEE_ABSTRACT_PROTOCOL_H
