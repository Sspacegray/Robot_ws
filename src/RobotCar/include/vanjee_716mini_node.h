#ifndef VANJEE_716MINI_NODE_H
#define VANJEE_716MINI_NODE_H

#include "vanjee_716mini_lidar_protocol.h"
#include "vanjee_common_tcp.h"
#include "vanjee_common_udp.h"
#include <dynamic_reconfigure/server.h>

namespace vanjee_lidar
{
    class Vanjee716MiniNode
    {
    public:
        Vanjee716MiniNode();
        ~Vanjee716MiniNode();
        void callback(RobotCar::vanjee_716mini_lidarConfig &config, uint32_t level);
        void connect();
        void checkConnection();

    private:
        Vanjee716MiniLidarProtocol *lidar_protocol_;
        std::string lidar_ip_;
        int lidar_port_;
        std::string host_ip_;
        int host_port_;
        int communication_mode_;
        VanjeeCommonAbstract *commumication_;
        dynamic_reconfigure::Server<RobotCar::vanjee_716mini_lidarConfig> server_;
    };

}
#endif // VANJEE_716MINI_NODE_H
