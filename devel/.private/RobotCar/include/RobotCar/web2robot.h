// Generated by gencpp from file RobotCar/web2robot.msg
// DO NOT EDIT!


#ifndef ROBOTCAR_MESSAGE_WEB2ROBOT_H
#define ROBOTCAR_MESSAGE_WEB2ROBOT_H

#include <ros/service_traits.h>


#include <RobotCar/web2robotRequest.h>
#include <RobotCar/web2robotResponse.h>


namespace RobotCar
{

struct web2robot
{

typedef web2robotRequest Request;
typedef web2robotResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct web2robot
} // namespace RobotCar


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::RobotCar::web2robot > {
  static const char* value()
  {
    return "1b7da60caa7293bcfbc236579ba5c0d8";
  }

  static const char* value(const ::RobotCar::web2robot&) { return value(); }
};

template<>
struct DataType< ::RobotCar::web2robot > {
  static const char* value()
  {
    return "RobotCar/web2robot";
  }

  static const char* value(const ::RobotCar::web2robot&) { return value(); }
};


// service_traits::MD5Sum< ::RobotCar::web2robotRequest> should match
// service_traits::MD5Sum< ::RobotCar::web2robot >
template<>
struct MD5Sum< ::RobotCar::web2robotRequest>
{
  static const char* value()
  {
    return MD5Sum< ::RobotCar::web2robot >::value();
  }
  static const char* value(const ::RobotCar::web2robotRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::RobotCar::web2robotRequest> should match
// service_traits::DataType< ::RobotCar::web2robot >
template<>
struct DataType< ::RobotCar::web2robotRequest>
{
  static const char* value()
  {
    return DataType< ::RobotCar::web2robot >::value();
  }
  static const char* value(const ::RobotCar::web2robotRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::RobotCar::web2robotResponse> should match
// service_traits::MD5Sum< ::RobotCar::web2robot >
template<>
struct MD5Sum< ::RobotCar::web2robotResponse>
{
  static const char* value()
  {
    return MD5Sum< ::RobotCar::web2robot >::value();
  }
  static const char* value(const ::RobotCar::web2robotResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::RobotCar::web2robotResponse> should match
// service_traits::DataType< ::RobotCar::web2robot >
template<>
struct DataType< ::RobotCar::web2robotResponse>
{
  static const char* value()
  {
    return DataType< ::RobotCar::web2robot >::value();
  }
  static const char* value(const ::RobotCar::web2robotResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROBOTCAR_MESSAGE_WEB2ROBOT_H
