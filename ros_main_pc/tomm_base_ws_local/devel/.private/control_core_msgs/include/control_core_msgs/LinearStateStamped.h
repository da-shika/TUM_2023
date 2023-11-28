// Generated by gencpp from file control_core_msgs/LinearStateStamped.msg
// DO NOT EDIT!


#ifndef CONTROL_CORE_MSGS_MESSAGE_LINEARSTATESTAMPED_H
#define CONTROL_CORE_MSGS_MESSAGE_LINEARSTATESTAMPED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <control_core_msgs/LinearState.h>

namespace control_core_msgs
{
template <class ContainerAllocator>
struct LinearStateStamped_
{
  typedef LinearStateStamped_<ContainerAllocator> Type;

  LinearStateStamped_()
    : header()
    , state()  {
    }
  LinearStateStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::control_core_msgs::LinearState_<ContainerAllocator>  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> const> ConstPtr;

}; // struct LinearStateStamped_

typedef ::control_core_msgs::LinearStateStamped_<std::allocator<void> > LinearStateStamped;

typedef boost::shared_ptr< ::control_core_msgs::LinearStateStamped > LinearStateStampedPtr;
typedef boost::shared_ptr< ::control_core_msgs::LinearStateStamped const> LinearStateStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_core_msgs::LinearStateStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::control_core_msgs::LinearStateStamped_<ContainerAllocator1> & lhs, const ::control_core_msgs::LinearStateStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.state == rhs.state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::control_core_msgs::LinearStateStamped_<ContainerAllocator1> & lhs, const ::control_core_msgs::LinearStateStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace control_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b1b7305065501d4951b8d88b820348e4";
  }

  static const char* value(const ::control_core_msgs::LinearStateStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb1b7305065501d49ULL;
  static const uint64_t static_value2 = 0x51b8d88b820348e4ULL;
};

template<class ContainerAllocator>
struct DataType< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_core_msgs/LinearStateStamped";
  }

  static const char* value(const ::control_core_msgs::LinearStateStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"LinearState state\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: control_core_msgs/LinearState\n"
"geometry_msgs/Point position\n"
"geometry_msgs/Vector3 velocity\n"
"geometry_msgs/Vector3 acceleration\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::control_core_msgs::LinearStateStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LinearStateStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_core_msgs::LinearStateStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_core_msgs::LinearStateStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    s << std::endl;
    Printer< ::control_core_msgs::LinearState_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_CORE_MSGS_MESSAGE_LINEARSTATESTAMPED_H