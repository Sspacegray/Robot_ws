#ifndef VANJEE_COMMON_UDP_H
#define VANJEE_COMMON_UDP_H

#include <boost/scoped_ptr.hpp>
#include "vanjee_abstract_protocol.h"
#include "vanjee_common_abstract.h"

namespace vanjee_lidar
{
    class VanjeeCommonUdp : public VanjeeCommonAbstract
    {
    public:
        VanjeeCommonUdp(std::string host_ip, int host_port, VanjeeAbstractProtocol *protocol);
        ~VanjeeCommonUdp();
        bool connect(std::string lidar_ip, int lidar_port);
        bool disconnect();
        void startIoService();
        void stopIoService();
        void handleRead(boost::system::error_code error, size_t bytes_count);
        void reconnect();
        bool sendData(unsigned char buf[], int length);

    private:
        VanjeeAbstractProtocol *protocol_;
        std::string host_ip_;
        int host_port_;
        std::string lidar_ip_;
        int lidar_port_;
        boost::asio::io_service io_;
        boost::scoped_ptr<boost::thread> io_run_thread_;
        boost::scoped_ptr<boost::asio::io_service::work> virtual_work_;
        boost::asio::ip::udp::endpoint udp_endpoint_;
        boost::shared_ptr<boost::asio::ip::udp::socket> udp_socket_;

        unsigned char receive_buffer_[MAX_LENGTH];
    };

}
#endif // VANJEE_COMMON_UDP_H
