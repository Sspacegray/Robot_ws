#ifndef VANJEE_718MINI_LIDAR_PROTOCOL_H
#define VANJEE_718MINI_LIDAR_PROTOCOL_H

#include "vanjee_abstract_protocol.h"
#include <vanjee_mini_lidar/vanjee_718mini_lidarConfig.h>
#include <set>

namespace vanjee_lidar
{
    class Vanjee718MiniLidarProtocol : public VanjeeAbstractProtocol
    {
    public:
        Vanjee718MiniLidarProtocol();
        ~Vanjee718MiniLidarProtocol();
        bool protocl(unsigned char *data, const int len);
        // void sendScan(const char *data,const int len);
        bool setConfig(vanjee_mini_lidar::vanjee_718mini_lidarConfig &new_config, uint32_t level);
        bool heartstate_;

    private:
        vanjee_mini_lidar::vanjee_718mini_lidarConfig config_;
        unsigned int pre_frame_no_;
        std::set<int> received_package_no_;
        float scan_data_[1081];
        float scan_intensity_[1081];
        int index_start_;
        int index_end_;
        int freq_scan_;
    };

}
#endif
