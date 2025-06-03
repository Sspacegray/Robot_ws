#include "vanjee_mini_lidar/vanjee_716mini_lidar_protocol.h"

namespace vanjee_lidar
{
    Vanjee716MiniLidarProtocol::Vanjee716MiniLidarProtocol()
    {
        freq_scan_ = 1;
        pre_frame_no_ = 0;

        scan_.header.frame_id = "vanjee_716mini_lidar_frame";
        scan_.angle_min = -2.35619449;
        scan_.angle_max = 2.35619449;
        scan_.angle_increment = 0.017453 / 4;
        scan_.time_increment = 1 / 15.00000000 / 1440;
        scan_.range_min = 0;
        scan_.range_max = 30;
        scan_.ranges.resize(1081);
        scan_.intensities.resize(1081);

        std::cout << "vanjee_716mini_protocl start success" << std::endl;
    }

    Vanjee716MiniLidarProtocol::~Vanjee716MiniLidarProtocol()
    {

    }

    bool Vanjee716MiniLidarProtocol::setConfig(vanjee_mini_lidar::vanjee_716mini_lidarConfig &new_config, uint32_t level)
    {
        config_ = new_config;
        scan_.header.frame_id = config_.frame_id;
        scan_.angle_min = config_.min_ang;
        scan_.angle_max = config_.max_ang;
        scan_.range_min = config_.range_min;
        scan_.range_max = config_.range_max;
        freq_scan_ = config_.frequency_scan;

        scan_.angle_increment = 0.017453 / 4;
        if (freq_scan_ == 1) //0.25°_15hz
        {
            scan_.time_increment = 1 / 15.00000000 / 1440;
        }
        else if (freq_scan_ == 2) //0.25°_25hz
        {
            scan_.time_increment = 1 / 25.00000000 / 1440;
        }

        // adjust angle_min to min_ang config param
        index_start_ = (config_.min_ang + 2.35619449) / scan_.angle_increment;
        // adjust angle_max to max_ang config param
        index_end_ = 1081 - ((2.35619449 - config_.max_ang) / scan_.angle_increment);
        int samples = index_end_ - index_start_;
        scan_.ranges.resize(samples);
        scan_.intensities.resize(samples);

        std::cout << "frame_id:" << scan_.header.frame_id << std::endl;
        std::cout << "min_ang:" << scan_.angle_min << std::endl;
        std::cout << "max_ang:" << scan_.angle_max << std::endl;
        std::cout << "angle_increment:" << scan_.angle_increment << std::endl;
        std::cout << "time_increment:" << scan_.time_increment << std::endl;
        std::cout << "range_min:" << scan_.range_min << std::endl;
        std::cout << "range_max:" << scan_.range_max << std::endl;
        std::cout << "samples_per_scan:" << samples << std::endl;
        return true;
    }

    bool Vanjee716MiniLidarProtocol::protocl(unsigned char *data, const int len)
    {
        if (data[13] != 0x0D)
        {
            return false;
        }
        if ((data[22] == 0x02 && data[23] == 0x02) || (data[22] == 0x02 && data[23] == 0x01))
        {
            heartstate_ = true;
            int l_n32TotalPackage = data[80];
            int l_n32PackageNo = data[81];
            unsigned int l_u32FrameNo = (data[75] << 24) + (data[76] << 16) + (data[77] << 8) + data[78];
            int l_n32PointNum = (data[83] << 8) + data[84];
            int l_n32Frequency = data[79];
            int l_n32StartIndex = 0;

            if (l_n32Frequency != freq_scan_)
            {
                std::cout << "The scan frequency does not match the one you setted!" << std::endl;
                return false;
            }

            if (pre_frame_no_ != l_u32FrameNo)
            {
                pre_frame_no_ = l_u32FrameNo;
                received_package_no_.clear();
                memset(scan_intensity_,0,sizeof(scan_intensity_));
            }

            received_package_no_.insert(l_n32PackageNo);

            if (data[82] == 0x00) //Dist
            {
                l_n32StartIndex = 600 * (l_n32PackageNo - 1);
                for (int j = 0; j < l_n32PointNum; j++)
                {
                    scan_data_[l_n32StartIndex + j] = (((unsigned char)data[85 + j * 2]) << 8) +
                                                      ((unsigned char)data[86 + j * 2]);
                    scan_data_[l_n32StartIndex + j] /= 1000.0;
                    if (scan_data_[l_n32StartIndex + j] > scan_.range_max || scan_data_[l_n32StartIndex + j] < scan_.range_min || scan_data_[l_n32StartIndex + j] == 0)
                    {
                        scan_data_[l_n32StartIndex + j] = NAN;
                    }
                }
            }
            else if (data[82] == 0x01) //intensities
            {
                l_n32StartIndex = 600 * (l_n32PackageNo - 3);
                for (int j = 0; j < l_n32PointNum; j++)
                {
                    scan_intensity_[l_n32StartIndex + j] = (((unsigned char)data[85 + j * 2]) << 8) +
                                                           ((unsigned char)data[86 + j * 2]);
                }
            }

            if (received_package_no_.size() == l_n32TotalPackage)
            {
                for (int i = index_start_; i < index_end_; i++)
                {
                    scan_.ranges[i - index_start_] = scan_data_[i];
                    if (scan_data_[i - index_start_] == NAN)
                    {
                        scan_.intensities[i - index_start_] = 0;
                    }
                    else
                    {
                        scan_.intensities[i - index_start_] = scan_intensity_[i];
                    }
                }

                ros::Time scan_time = ros::Time::now();
                scan_.header.stamp = scan_time;
                marker_pub_.publish(scan_);
            }

            return true;
        }
        else
        {
            return false;
        }
    }
}
