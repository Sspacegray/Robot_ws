#include "vanjee_mini_lidar/vanjee_common_udp.h"

namespace vanjee_lidar
{
    VanjeeCommonUdp::VanjeeCommonUdp(std::string host_ip, int host_port, VanjeeAbstractProtocol *protocol)
    {
        protocol_ = protocol;
        host_ip_ = host_ip;
        host_port_ = host_port;
    }

    VanjeeCommonUdp::~VanjeeCommonUdp()
    {
        disconnect();
    }

    bool VanjeeCommonUdp::connect(std::string lidar_ip, int lidar_port)
    {
        try
        {
            if (lidar_ip.empty())
            {
                return false;
            }
            else
            {
                lidar_ip_ = lidar_ip;
                lidar_port_ = lidar_port;
            }

            if (udp_socket_ && udp_socket_->is_open())
            {
                boost::system::error_code errorcode;
                udp_socket_->shutdown(boost::asio::ip::udp::socket::shutdown_both, errorcode);
                udp_socket_->close();
            }

            boost::asio::ip::udp::endpoint receive_endpoint(boost::asio::ip::udp::v4(), host_port_);
            receive_endpoint.address(boost::asio::ip::address::from_string(host_ip_));
            udp_socket_ = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(io_, receive_endpoint));

            connected_ = true;
            startIoService();

            udp_socket_->async_receive_from(boost::asio::buffer(receive_buffer_), udp_endpoint_,
                                            boost::bind(&VanjeeCommonUdp::handleRead, this,
                                                        boost::asio::placeholders::error,
                                                        boost::asio::placeholders::bytes_transferred));

            return true;

        }
        catch (boost::exception &e)
        {
            connected_ = false;
            return false;
        }
    }

    bool VanjeeCommonUdp::disconnect()
    {
        try
        {
            stopIoService();
            std::cout << "Closing UDP!" << std::endl;
            connected_ = false;
            if (udp_socket_ && udp_socket_->is_open())
            {
                boost::system::error_code error_code;
                udp_socket_->shutdown(boost::asio::ip::udp::socket::shutdown_both, error_code);
                udp_socket_->close();
            }
            
            return true;
        }
        catch (boost::exception &e)
        {
            return false;
        }
    }

    void VanjeeCommonUdp::startIoService()
    {
        virtual_work_.reset(new boost::asio::io_service::work(io_));
        if (io_run_thread_)
            return;
        this->io_run_thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &this->io_)));
    }

    void VanjeeCommonUdp::stopIoService()
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

    void VanjeeCommonUdp::handleRead(boost::system::error_code error, size_t bytes_count)
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

        udp_socket_->async_receive_from(boost::asio::buffer(receive_buffer_), udp_endpoint_,
                                        boost::bind(&VanjeeCommonUdp::handleRead, this,
                                                    boost::asio::placeholders::error,
                                                    boost::asio::placeholders::bytes_transferred));

    }

    void VanjeeCommonUdp::reconnect()
    {
        reconnecting_ = true;
        while (!connected_)
        {
            if (udp_socket_ && udp_socket_->is_open())
            {
                boost::system::error_code error_code;
                udp_socket_->shutdown(boost::asio::ip::udp::socket::shutdown_both, error_code);
                udp_socket_->close();
                std::cout << "Initializing network!" << std::endl;
            }
            sleep(1);
            std::cout << "Start opening UDP!" << std::endl;
            sleep(1);
            if (connect(lidar_ip_, lidar_port_))
            {
                std::cout << "Succesfully opened!" << std::endl;
                break;
            }
            else
            {
                std::cout << "Failed to reopen!" << std::endl;
            }
            sleep(1);
        }
        reconnecting_ = false;
    }

    bool VanjeeCommonUdp::sendData(unsigned char buf[], int length)
    {
        try
        {
            if (udp_socket_ && connected_)
            {
                boost::asio::ip::udp::endpoint sendEndpoint(boost::asio::ip::udp::v4(), lidar_port_);
                sendEndpoint.address(boost::asio::ip::address::from_string(lidar_ip_));
                size_t st = udp_socket_->send_to(boost::asio::buffer(buf, length), sendEndpoint);
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
