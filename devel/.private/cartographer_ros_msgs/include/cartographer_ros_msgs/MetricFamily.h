// Generated by gencpp from file cartographer_ros_msgs/MetricFamily.msg
// DO NOT EDIT!


#ifndef CARTOGRAPHER_ROS_MSGS_MESSAGE_METRICFAMILY_H
#define CARTOGRAPHER_ROS_MSGS_MESSAGE_METRICFAMILY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <cartographer_ros_msgs/Metric.h>

namespace cartographer_ros_msgs
{
template <class ContainerAllocator>
struct MetricFamily_
{
  typedef MetricFamily_<ContainerAllocator> Type;

  MetricFamily_()
    : name()
    , description()
    , metrics()  {
    }
  MetricFamily_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , description(_alloc)
    , metrics(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_type;
  _name_type name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _description_type;
  _description_type description;

   typedef std::vector< ::cartographer_ros_msgs::Metric_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::cartographer_ros_msgs::Metric_<ContainerAllocator> >> _metrics_type;
  _metrics_type metrics;





  typedef boost::shared_ptr< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> const> ConstPtr;

}; // struct MetricFamily_

typedef ::cartographer_ros_msgs::MetricFamily_<std::allocator<void> > MetricFamily;

typedef boost::shared_ptr< ::cartographer_ros_msgs::MetricFamily > MetricFamilyPtr;
typedef boost::shared_ptr< ::cartographer_ros_msgs::MetricFamily const> MetricFamilyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator1> & lhs, const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name &&
    lhs.description == rhs.description &&
    lhs.metrics == rhs.metrics;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator1> & lhs, const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cartographer_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
{
  static const char* value()
  {
    return "583a11b161bb4a70f5df274715bcaf10";
  }

  static const char* value(const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x583a11b161bb4a70ULL;
  static const uint64_t static_value2 = 0xf5df274715bcaf10ULL;
};

template<class ContainerAllocator>
struct DataType< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartographer_ros_msgs/MetricFamily";
  }

  static const char* value(const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# 2018 The Cartographer Authors\n"
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
"string name\n"
"string description\n"
"cartographer_ros_msgs/Metric[] metrics\n"
"\n"
"================================================================================\n"
"MSG: cartographer_ros_msgs/Metric\n"
"# 2018 The Cartographer Authors\n"
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
"uint8 TYPE_COUNTER=0\n"
"uint8 TYPE_GAUGE=1\n"
"uint8 TYPE_HISTOGRAM=2\n"
"\n"
"uint8 type\n"
"cartographer_ros_msgs/MetricLabel[] labels\n"
"\n"
"# TYPE_COUNTER or TYPE_GAUGE\n"
"float64 value\n"
"\n"
"# TYPE_HISTOGRAM\n"
"cartographer_ros_msgs/HistogramBucket[] counts_by_bucket\n"
"\n"
"================================================================================\n"
"MSG: cartographer_ros_msgs/MetricLabel\n"
"# 2018 The Cartographer Authors\n"
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
"string key\n"
"string value\n"
"\n"
"================================================================================\n"
"MSG: cartographer_ros_msgs/HistogramBucket\n"
"# 2018 The Cartographer Authors\n"
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
"# A histogram bucket counts values x for which:\n"
"#   previous_boundary < x <= bucket_boundary\n"
"# holds.\n"
"float64 bucket_boundary\n"
"float64 count\n"
;
  }

  static const char* value(const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.description);
      stream.next(m.metrics);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MetricFamily_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cartographer_ros_msgs::MetricFamily_<ContainerAllocator>& v)
  {
    if (false || !indent.empty())
      s << std::endl;
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.description);
    if (true || !indent.empty())
      s << std::endl;
    s << indent << "metrics: ";
    if (v.metrics.empty() || false)
      s << "[";
    for (size_t i = 0; i < v.metrics.size(); ++i)
    {
      if (false && i > 0)
        s << ", ";
      else if (!false)
        s << std::endl << indent << "  -";
      Printer< ::cartographer_ros_msgs::Metric_<ContainerAllocator> >::stream(s, false ? std::string() : indent + "    ", v.metrics[i]);
    }
    if (v.metrics.empty() || false)
      s << "]";
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARTOGRAPHER_ROS_MSGS_MESSAGE_METRICFAMILY_H
