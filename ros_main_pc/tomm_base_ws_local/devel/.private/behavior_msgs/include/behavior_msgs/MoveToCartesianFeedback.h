// Generated by gencpp from file behavior_msgs/MoveToCartesianFeedback.msg
// DO NOT EDIT!


#ifndef BEHAVIOR_MSGS_MESSAGE_MOVETOCARTESIANFEEDBACK_H
#define BEHAVIOR_MSGS_MESSAGE_MOVETOCARTESIANFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace behavior_msgs
{
template <class ContainerAllocator>
struct MoveToCartesianFeedback_
{
  typedef MoveToCartesianFeedback_<ContainerAllocator> Type;

  MoveToCartesianFeedback_()
    : cmd()
    , real()  {
    }
  MoveToCartesianFeedback_(const ContainerAllocator& _alloc)
    : cmd(_alloc)
    , real(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _cmd_type;
  _cmd_type cmd;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _real_type;
  _real_type real;





  typedef boost::shared_ptr< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct MoveToCartesianFeedback_

typedef ::behavior_msgs::MoveToCartesianFeedback_<std::allocator<void> > MoveToCartesianFeedback;

typedef boost::shared_ptr< ::behavior_msgs::MoveToCartesianFeedback > MoveToCartesianFeedbackPtr;
typedef boost::shared_ptr< ::behavior_msgs::MoveToCartesianFeedback const> MoveToCartesianFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator1> & lhs, const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.cmd == rhs.cmd &&
    lhs.real == rhs.real;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator1> & lhs, const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace behavior_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5dffad9e3fc5177a0ee928b18ee386b4";
  }

  static const char* value(const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5dffad9e3fc5177aULL;
  static const uint64_t static_value2 = 0x0ee928b18ee386b4ULL;
};

template<class ContainerAllocator>
struct DataType< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "behavior_msgs/MoveToCartesianFeedback";
  }

  static const char* value(const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"geometry_msgs/PoseStamped cmd\n"
"geometry_msgs/PoseStamped real\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
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
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cmd);
      stream.next(m.real);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveToCartesianFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::behavior_msgs::MoveToCartesianFeedback_<ContainerAllocator>& v)
  {
    s << indent << "cmd: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.cmd);
    s << indent << "real: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.real);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEHAVIOR_MSGS_MESSAGE_MOVETOCARTESIANFEEDBACK_H
