// Generated by gencpp from file cartographer_ros_msgs/FinishTrajectoryResponse.msg
// DO NOT EDIT!


#ifndef CARTOGRAPHER_ROS_MSGS_MESSAGE_FINISHTRAJECTORYRESPONSE_H
#define CARTOGRAPHER_ROS_MSGS_MESSAGE_FINISHTRAJECTORYRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <cartographer_ros_msgs/StatusResponse.h>

namespace cartographer_ros_msgs
{
template <class ContainerAllocator>
struct FinishTrajectoryResponse_
{
  typedef FinishTrajectoryResponse_<ContainerAllocator> Type;

  FinishTrajectoryResponse_()
    : status()  {
    }
  FinishTrajectoryResponse_(const ContainerAllocator& _alloc)
    : status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::cartographer_ros_msgs::StatusResponse_<ContainerAllocator>  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> const> ConstPtr;

}; // struct FinishTrajectoryResponse_

typedef ::cartographer_ros_msgs::FinishTrajectoryResponse_<std::allocator<void> > FinishTrajectoryResponse;

typedef boost::shared_ptr< ::cartographer_ros_msgs::FinishTrajectoryResponse > FinishTrajectoryResponsePtr;
typedef boost::shared_ptr< ::cartographer_ros_msgs::FinishTrajectoryResponse const> FinishTrajectoryResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator1> & lhs, const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator2> & rhs)
{
  return lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator1> & lhs, const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cartographer_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4e6ca4e44081fa06b258fa12804ea7cb";
  }

  static const char* value(const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4e6ca4e44081fa06ULL;
  static const uint64_t static_value2 = 0xb258fa12804ea7cbULL;
};

template<class ContainerAllocator>
struct DataType< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartographer_ros_msgs/FinishTrajectoryResponse";
  }

  static const char* value(const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartographer_ros_msgs/StatusResponse status\n"
"\n"
"\n"
"================================================================================\n"
"MSG: cartographer_ros_msgs/StatusResponse\n"
"# Copyright 2018 The Cartographer Authors\n"
"#\n"
"# Licensed under the Apache License, Version 2.0 (the \"License\");\n"
"# you may not use this file except in compliance with the License.\n"
"# You may obtain a copy of the License at\n"
"#\n"
"#      http://www.apache.org/licenses/LICENSE-2.0\n"
"#\n"
"# Unless required by applicable law or agreed to in writing, software\n"
"# distributed under the License is distributed on an \"AS IS\" BASIS,\n"
"# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
"# See the License for the specific language governing permissions and\n"
"# limitations under the License.\n"
"\n"
"# A common message type to indicate the outcome of a service call.\n"
"uint8 code\n"
"string message\n"
;
  }

  static const char* value(const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FinishTrajectoryResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cartographer_ros_msgs::FinishTrajectoryResponse_<ContainerAllocator>& v)
  {
    if (false || !indent.empty())
      s << std::endl;
    s << indent << "status: ";
    Printer< ::cartographer_ros_msgs::StatusResponse_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARTOGRAPHER_ROS_MSGS_MESSAGE_FINISHTRAJECTORYRESPONSE_H
