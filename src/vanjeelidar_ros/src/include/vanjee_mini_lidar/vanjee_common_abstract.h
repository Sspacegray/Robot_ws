#ifndef VANJEE_COMMON_ABSTRACT_H
#define VANJEE_COMMON_ABSTRACT_H

//#include <iostream>
//#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>

namespace vanjee_lidar
{
#define MAX_LENGTH 50000

    class VanjeeCommonAbstract
    {
    public:
        VanjeeCommonAbstract()
        {
            connected_ = false;
            reconnecting_ = false;
        };
        virtual ~VanjeeCommonAbstract(){};
        virtual bool connect(std::string lidar_ip, int lidar_port) = 0;
        virtual bool disconnect() = 0;
        virtual void startIoService() = 0;
        virtual void stopIoService() = 0;
        virtual void handleRead(boost::system::error_code error, size_t bytes_count) = 0;
        virtual void reconnect() = 0;
        virtual bool sendData(unsigned char buf[], int length) = 0;
        bool connected_;
        bool reconnecting_;
    };
}
#endif // VANJEE_COMMON_ABSTRACT_H
