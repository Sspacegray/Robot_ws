// Generated by gencpp from file cartographer_ros_msgs/BagfileProgress.msg
// DO NOT EDIT!


#ifndef CARTOGRAPHER_ROS_MSGS_MESSAGE_BAGFILEPROGRESS_H
#define CARTOGRAPHER_ROS_MSGS_MESSAGE_BAGFILEPROGRESS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cartographer_ros_msgs
{
template <class ContainerAllocator>
struct BagfileProgress_
{
  typedef BagfileProgress_<ContainerAllocator> Type;

  BagfileProgress_()
    : current_bagfile_name()
    , current_bagfile_id(0)
    , total_bagfiles(0)
    , total_messages(0)
    , processed_messages(0)
    , total_seconds(0.0)
    , processed_seconds(0.0)  {
    }
  BagfileProgress_(const ContainerAllocator& _alloc)
    : current_bagfile_name(_alloc)
    , current_bagfile_id(0)
    , total_bagfiles(0)
    , total_messages(0)
    , processed_messages(0)
    , total_seconds(0.0)
    , processed_seconds(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _current_bagfile_name_type;
  _current_bagfile_name_type current_bagfile_name;

   typedef uint32_t _current_bagfile_id_type;
  _current_bagfile_id_type current_bagfile_id;

   typedef uint32_t _total_bagfiles_type;
  _total_bagfiles_type total_bagfiles;

   typedef uint32_t _total_messages_type;
  _total_messages_type total_messages;

   typedef uint32_t _processed_messages_type;
  _processed_messages_type processed_messages;

   typedef float _total_seconds_type;
  _total_seconds_type total_seconds;

   typedef float _processed_seconds_type;
  _processed_seconds_type processed_seconds;





  typedef boost::shared_ptr< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> const> ConstPtr;

}; // struct BagfileProgress_

typedef ::cartographer_ros_msgs::BagfileProgress_<std::allocator<void> > BagfileProgress;

typedef boost::shared_ptr< ::cartographer_ros_msgs::BagfileProgress > BagfileProgressPtr;
typedef boost::shared_ptr< ::cartographer_ros_msgs::BagfileProgress const> BagfileProgressConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator1> & lhs, const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator2> & rhs)
{
  return lhs.current_bagfile_name == rhs.current_bagfile_name &&
    lhs.current_bagfile_id == rhs.current_bagfile_id &&
    lhs.total_bagfiles == rhs.total_bagfiles &&
    lhs.total_messages == rhs.total_messages &&
    lhs.processed_messages == rhs.processed_messages &&
    lhs.total_seconds == rhs.total_seconds &&
    lhs.processed_seconds == rhs.processed_seconds;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator1> & lhs, const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cartographer_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2a36f93b13e2b297d45098a38cb00510";
  }

  static const char* value(const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2a36f93b13e2b297ULL;
  static const uint64_t static_value2 = 0xd45098a38cb00510ULL;
};

template<class ContainerAllocator>
struct DataType< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartographer_ros_msgs/BagfileProgress";
  }

  static const char* value(const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Licensed under the Apache License, Version 2.0 (the 'License');\n"
"# you may not use this file except in compliance with the License.\n"
"# You may obtain a copy of the License at\n"
"#\n"
"#      http://www.apache.org/licenses/LICENSE-2.0\n"
"#\n"
"# Unless required by applicable law or agreed to in writing, software\n"
"# distributed under the License is distributed on an 'AS IS' BASIS,\n"
"# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
"# See the License for the specific language governing permissions and\n"
"# limitations under the License.\n"
"\n"
"\n"
"# Contains general information about the bagfiles processing progress\n"
"\n"
"string current_bagfile_name\n"
"uint32 current_bagfile_id\n"
"uint32 total_bagfiles\n"
"uint32 total_messages\n"
"uint32 processed_messages\n"
"float32 total_seconds\n"
"float32 processed_seconds\n"
;
  }

  static const char* value(const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current_bagfile_name);
      stream.next(m.current_bagfile_id);
      stream.next(m.total_bagfiles);
      stream.next(m.total_messages);
      stream.next(m.processed_messages);
      stream.next(m.total_seconds);
      stream.next(m.processed_seconds);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BagfileProgress_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cartographer_ros_msgs::BagfileProgress_<ContainerAllocator>& v)
  {
    if (false || !indent.empty())
      s << std::endl;
    s << indent << "current_bagfile_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.current_bagfile_name);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "current_bagfile_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.current_bagfile_id);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "total_bagfiles: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.total_bagfiles);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "total_messages: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.total_messages);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "processed_messages: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.processed_messages);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "total_seconds: ";
    Printer<float>::stream(s, indent + "  ", v.total_seconds);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "processed_seconds: ";
    Printer<float>::stream(s, indent + "  ", v.processed_seconds);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARTOGRAPHER_ROS_MSGS_MESSAGE_BAGFILEPROGRESS_H
