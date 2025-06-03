#ifndef VANJEE_718MINI_NODE_H
#define VANJEE_718MINI_NODE_H

#include "vanjee_718mini_lidar_protocol.h"
#include "vanjee_common_tcp.h"
#include "vanjee_common_udp.h"
#include <dynamic_reconfigure/server.h>

namespace vanjee_lidar
{
    class Vanjee718MiniNode
    {
    public:
        Vanjee718MiniNode();
        ~Vanjee718MiniNode();
        void callback(vanjee_mini_lidar::vanjee_718mini_lidarConfig &config, uint32_t level);
        void connect();
        void checkConnection();

    private:
        Vanjee718MiniLidarProtocol *lidar_protocol_;
        std::string lidar_ip_;
        int lidar_port_;
        std::string host_ip_;
        int host_port_;
        int communication_mode_;
        VanjeeCommonAbstract *commumication_;
        dynamic_reconfigure::Server<vanjee_mini_lidar::vanjee_718mini_lidarConfig> server_;
    };

}
#endif // VANJEE_718MINI_NODE_H
