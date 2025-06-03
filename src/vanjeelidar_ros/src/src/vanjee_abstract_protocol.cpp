#include "vanjee_mini_lidar/vanjee_abstract_protocol.h"

namespace vanjee_lidar
{
    VanjeeAbstractProtocol::VanjeeAbstractProtocol()
    {
        memset(&data_, 0, sizeof(data_));
        marker_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 50);
        ros::Time scan_time = ros::Time::now();
        scan_.header.stamp = scan_time;
    }

    bool VanjeeAbstractProtocol::dataProcess(unsigned char *data, const int reclen)
    {
        if (reclen > MAX_LENGTH_DATA_PROCESS)
        {
            data_.cache_out = 0;
            data_.cache_in = 0;
            return false;
        }

        if (data_.cache_in + reclen > MAX_LENGTH_DATA_PROCESS)
        {
            data_.cache_out = 0;
            data_.cache_in = 0;
            return false;
        }
        memcpy(&data_.cache_data[data_.cache_in], data, reclen * sizeof(char));
        data_.cache_in += reclen;
        while (data_.cache_out < data_.cache_in)
        {
            if (data_.cache_data[data_.cache_out] == 0xFF && data_.cache_data[data_.cache_out + 1] == 0xAA)
            {
                unsigned l_u32reallen = (data_.cache_data[data_.cache_out + 2] << 8) |
                                        (data_.cache_data[data_.cache_out + 3] << 0);
                l_u32reallen = l_u32reallen + 4;

                if (l_u32reallen <= (data_.cache_in - data_.cache_out + 1))
                {
                    if (OnRecvProcess(&data_.cache_data[data_.cache_out], l_u32reallen))
                    {
                        data_.cache_out += l_u32reallen;
                    }
                    else
                    {
                        std::cout << "continuous search frame header" << std::endl;
                        data_.cache_out++;
                    }
                }
                else if (l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
                {
                    data_.cache_out++;
                }
                else
                {
                    break;
                }
            }
            else
            {
                data_.cache_out++;
            }
        } //end while(data_.cache_out < data_.cache_in)

        if (data_.cache_out >= data_.cache_in)
        {
            data_.cache_out = 0;
            data_.cache_in = 0;
        }
        else if (data_.cache_out < data_.cache_in && data_.cache_out != 0)
        {
            moveData(data_);
        }
        return true;
    }

    void VanjeeAbstractProtocol::moveData(DataCache &sdata)
    {
        for (int i = sdata.cache_out; i < sdata.cache_in; i++)
        {
            sdata.cache_data[i - sdata.cache_out] = sdata.cache_data[i];
        }
        sdata.cache_in = sdata.cache_in - sdata.cache_out;
        sdata.cache_out = 0;
    }

    bool VanjeeAbstractProtocol::OnRecvProcess(unsigned char *data, int len)
    {
        if (len > 0)
        {
            if (checkXor(data, len))
            {
                protocl(data, len);
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
        return true;
    }

    bool VanjeeAbstractProtocol::checkXor(unsigned char *recvbuf, int recvlen)
    {
        int i = 0;
        unsigned char check = 0;
        unsigned char *p = recvbuf;
        int len;
        if (*p == 0xFF)
        {
            p = p + 2;
            len = recvlen - 6;
            for (i = 0; i < len; i++)
            {
                check ^= *p++;
            }
            p++;
            if (check == *p)
            {
                return true;
            }
            else
                return false;
        }
        else
        {
            return false;
        }
    }

}
