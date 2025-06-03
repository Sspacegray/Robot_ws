#include "../include/vanjee_common_tcp.h"

namespace vanjee_lidar
{
    VanjeeCommonTcp::VanjeeCommonTcp(VanjeeAbstractProtocol *protocol)
    {
        protocol_ = protocol;
        tcp_socket_ = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(io_));
    }

    VanjeeCommonTcp::~VanjeeCommonTcp()
    {
        disconnect();
    }

    bool VanjeeCommonTcp::connect(std::string ip, int port)
    {
        try
        {
            if (connected_)
            {
                return false;
            }
            if (tcp_socket_->is_open())
            {
                boost::system::error_code errorcode;
                tcp_socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, errorcode);
                tcp_socket_->close();
            }
            lidar_ip_ = ip;
            lidar_port_ = port;
            ep_ = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port);
            tcp_socket_->connect(ep_, ec_);
            if (ec_)
            {
                connected_ = false;
                return false;
            }
            else
            {
                connected_ = true;
                startIoService();
                tcp_socket_->async_read_some(boost::asio::buffer(receive_buffer_),
                                             boost::bind(&VanjeeCommonTcp::handleRead, this,
                                                         boost::asio::placeholders::error,
                                                         boost::asio::placeholders::bytes_transferred));
                return true;
            }
        }
        catch (boost::exception &e)
        {
            return false;
        }
    }

    bool VanjeeCommonTcp::disconnect()
    {
        try
        {
            stopIoService();
            std::cout << "Disconnecting connection!" << std::endl;
            connected_ = false;
            if (tcp_socket_->is_open())
            {
                boost::system::error_code error_code;
                tcp_socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, error_code);
                tcp_socket_->close();
            }
            return true;
        }
        catch (boost::exception &e)
        {
            return false;
        }
    }

    void VanjeeCommonTcp::startIoService()
    {
        virtual_work_.reset(new boost::asio::io_service::work(io_));
        if (io_run_thread_)
            return;
        this->io_run_thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &this->io_)));
    }

    void VanjeeCommonTcp::stopIoService()
    {
        io_.stop();
        while (!io_.stopped())
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            io_.stop();
        }
        if (io_run_thread_)
        {
            io_run_thread_->join();
            io_run_thread_.reset();
        }
        virtual_work_.reset();
    }

    void VanjeeCommonTcp::handleRead(boost::system::error_code error, size_t bytes_count)
    {
        if (error)
        {
            connected_ = false;
            return;
        }

        if (bytes_count > 0)
        {
            protocol_->dataProcess(receive_buffer_, bytes_count);
        }

        tcp_socket_->async_read_some(boost::asio::buffer(receive_buffer_),
                                 boost::bind(&VanjeeCommonTcp::handleRead, this,
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));

    }

    void VanjeeCommonTcp::reconnect()
    {
        reconnecting_ = true;
        while (!connected_)
        {
            if (tcp_socket_->is_open())
            {
                boost::system::error_code error_code;
                tcp_socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, error_code);
                tcp_socket_->close();
                std::cout << "Initializing network!" << std::endl;
            }
            sleep(3);
            std::cout << "Start reconnecting laser!" << std::endl;
            sleep(2);
            if (connect(lidar_ip_, lidar_port_))
            {
                std::cout << "Succesfully connected!" << std::endl;
                break;
            }
            else
            {
                std::cout << "Failed to reconnect!" << std::endl;
            }
            sleep(2);
        }
        reconnecting_ = false;
    }

    bool VanjeeCommonTcp::sendData(unsigned char buf[], int length)
    {
        try
        {
            if (tcp_socket_->is_open() && connected_)
            {
                size_t st = tcp_socket_->send(boost::asio::buffer(buf, length));
                if (st == length)
                {
                    return true;
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
        }
        catch (std::exception e)
        {
            return false;
        }
    }

}
