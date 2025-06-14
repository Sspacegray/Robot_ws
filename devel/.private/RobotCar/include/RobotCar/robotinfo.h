// Generated by gencpp from file RobotCar/robotinfo.msg
// DO NOT EDIT!


#ifndef ROBOTCAR_MESSAGE_ROBOTINFO_H
#define ROBOTCAR_MESSAGE_ROBOTINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace RobotCar
{
template <class ContainerAllocator>
struct robotinfo_
{
  typedef robotinfo_<ContainerAllocator> Type;

  robotinfo_()
    : robotstate(0)
    , robotvoltage(0)
    , lastroompoint(0)  {
    }
  robotinfo_(const ContainerAllocator& _alloc)
    : robotstate(0)
    , robotvoltage(0)
    , lastroompoint(0)  {
  (void)_alloc;
    }



   typedef int32_t _robotstate_type;
  _robotstate_type robotstate;

   typedef int32_t _robotvoltage_type;
  _robotvoltage_type robotvoltage;

   typedef int32_t _lastroompoint_type;
  _lastroompoint_type lastroompoint;





  typedef boost::shared_ptr< ::RobotCar::robotinfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::RobotCar::robotinfo_<ContainerAllocator> const> ConstPtr;

}; // struct robotinfo_

typedef ::RobotCar::robotinfo_<std::allocator<void> > robotinfo;

typedef boost::shared_ptr< ::RobotCar::robotinfo > robotinfoPtr;
typedef boost::shared_ptr< ::RobotCar::robotinfo const> robotinfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::RobotCar::robotinfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::RobotCar::robotinfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::RobotCar::robotinfo_<ContainerAllocator1> & lhs, const ::RobotCar::robotinfo_<ContainerAllocator2> & rhs)
{
  return lhs.robotstate == rhs.robotstate &&
    lhs.robotvoltage == rhs.robotvoltage &&
    lhs.lastroompoint == rhs.lastroompoint;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::RobotCar::robotinfo_<ContainerAllocator1> & lhs, const ::RobotCar::robotinfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace RobotCar

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::RobotCar::robotinfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::RobotCar::robotinfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::RobotCar::robotinfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::RobotCar::robotinfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::RobotCar::robotinfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::RobotCar::robotinfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::RobotCar::robotinfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "00ba0d14f8e4b705d2bc45e5c870cf2b";
  }

  static const char* value(const ::RobotCar::robotinfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x00ba0d14f8e4b705ULL;
  static const uint64_t static_value2 = 0xd2bc45e5c870cf2bULL;
};

template<class ContainerAllocator>
struct DataType< ::RobotCar::robotinfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "RobotCar/robotinfo";
  }

  static const char* value(const ::RobotCar::robotinfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::RobotCar::robotinfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 robotstate\n"
"int32 robotvoltage\n"
"int32 lastroompoint\n"
;
  }

  static const char* value(const ::RobotCar::robotinfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::RobotCar::robotinfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robotstate);
      stream.next(m.robotvoltage);
      stream.next(m.lastroompoint);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct robotinfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::RobotCar::robotinfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::RobotCar::robotinfo_<ContainerAllocator>& v)
  {
    if (false || !indent.empty())
      s << std::endl;
    s << indent << "robotstate: ";
    Printer<int32_t>::stream(s, indent + "  ", v.robotstate);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "robotvoltage: ";
    Printer<int32_t>::stream(s, indent + "  ", v.robotvoltage);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "lastroompoint: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lastroompoint);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTCAR_MESSAGE_ROBOTINFO_H
