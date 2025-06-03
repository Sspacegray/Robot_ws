#ifndef VANJEE_COMMON_TCP_H
#define VANJEE_COMMON_TCP_H

#include <boost/scoped_ptr.hpp>
#include "vanjee_abstract_protocol.h"
#include "vanjee_common_abstract.h"

namespace vanjee_lidar
{
    class VanjeeCommonTcp : public VanjeeCommonAbstract
    {
    public:
        VanjeeCommonTcp(VanjeeAbstractProtocol *protocol);
        ~VanjeeCommonTcp();
        bool connect(std::string ip, int port);
        bool disconnect();
        void startIoService();
        void stopIoService();
        void handleRead(boost::system::error_code error, size_t bytes_count);
        void reconnect();
        bool sendData(unsigned char buf[], int length);

    private:
        VanjeeAbstractProtocol *protocol_;
        std::string lidar_ip_;
        int lidar_port_;
        boost::asio::io_service io_;
        boost::scoped_ptr<boost::thread> io_run_thread_;
        boost::scoped_ptr<boost::asio::io_service::work> virtual_work_;
        boost::asio::ip::tcp::endpoint ep_;
        boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_;
        boost::system::error_code ec_;

        unsigned char receive_buffer_[MAX_LENGTH];
    };

}
#endif // VANJEE_COMMON_TCP_H
