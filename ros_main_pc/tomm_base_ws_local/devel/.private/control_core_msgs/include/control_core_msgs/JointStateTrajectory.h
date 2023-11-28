// Generated by gencpp from file control_core_msgs/JointStateTrajectory.msg
// DO NOT EDIT!


#ifndef CONTROL_CORE_MSGS_MESSAGE_JOINTSTATETRAJECTORY_H
#define CONTROL_CORE_MSGS_MESSAGE_JOINTSTATETRAJECTORY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <control_core_msgs/JointState.h>

namespace control_core_msgs
{
template <class ContainerAllocator>
struct JointStateTrajectory_
{
  typedef JointStateTrajectory_<ContainerAllocator> Type;

  JointStateTrajectory_()
    : header()
    , time()
    , trajectory()  {
    }
  JointStateTrajectory_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time(_alloc)
    , trajectory(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _time_type;
  _time_type time;

   typedef std::vector< ::control_core_msgs::JointState_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::control_core_msgs::JointState_<ContainerAllocator> >> _trajectory_type;
  _trajectory_type trajectory;





  typedef boost::shared_ptr< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> const> ConstPtr;

}; // struct JointStateTrajectory_

typedef ::control_core_msgs::JointStateTrajectory_<std::allocator<void> > JointStateTrajectory;

typedef boost::shared_ptr< ::control_core_msgs::JointStateTrajectory > JointStateTrajectoryPtr;
typedef boost::shared_ptr< ::control_core_msgs::JointStateTrajectory const> JointStateTrajectoryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator1> & lhs, const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.time == rhs.time &&
    lhs.trajectory == rhs.trajectory;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator1> & lhs, const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace control_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ad80bd01b1a3c57c74c1c776dde6b612";
  }

  static const char* value(const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xad80bd01b1a3c57cULL;
  static const uint64_t static_value2 = 0x74c1c776dde6b612ULL;
};

template<class ContainerAllocator>
struct DataType< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_core_msgs/JointStateTrajectory";
  }

  static const char* value(const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"float64[] time\n"
"JointState[] trajectory\n"
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
"MSG: control_core_msgs/JointState\n"
"Vector position\n"
"Vector velocity\n"
"Vector acceleration\n"
"================================================================================\n"
"MSG: control_core_msgs/Vector\n"
"float64[] data\n"
;
  }

  static const char* value(const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time);
      stream.next(m.trajectory);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointStateTrajectory_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_core_msgs::JointStateTrajectory_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_core_msgs::JointStateTrajectory_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time[]" << std::endl;
    for (size_t i = 0; i < v.time.size(); ++i)
    {
      s << indent << "  time[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.time[i]);
    }
    s << indent << "trajectory[]" << std::endl;
    for (size_t i = 0; i < v.trajectory.size(); ++i)
    {
      s << indent << "  trajectory[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::control_core_msgs::JointState_<ContainerAllocator> >::stream(s, indent + "    ", v.trajectory[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_CORE_MSGS_MESSAGE_JOINTSTATETRAJECTORY_H