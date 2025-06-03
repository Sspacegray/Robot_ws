#ifndef VANJEE_718H_NODE_H
#define VANJEE_718H_NODE_H

#include "vanjee_718h_lidar_protocol.h"
#include "vanjee_common_tcp.h"
#include <dynamic_reconfigure/server.h>

namespace vanjee_lidar
{
    class Vanjee718HNode
    {
    public:
        Vanjee718HNode();
        ~Vanjee718HNode();
        void callback(vanjee_mini_lidar::vanjee_718h_lidarConfig &config, uint32_t level);
        void connect();
        void checkConnection();

    private:
        Vanjee718HLidarProtocol *lidar_protocol_;
        std::string lidar_ip_;
        int lidar_port_;
        std::string host_ip_;
        int host_port_;
        VanjeeCommonAbstract *commumication_;
        dynamic_reconfigure::Server<vanjee_mini_lidar::vanjee_718h_lidarConfig> server_;
    };

}
#endif // VANJEE_718H_NODE_H
