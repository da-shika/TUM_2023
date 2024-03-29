// Generated by gencpp from file walking_core_msgs/FootSteps.msg
// DO NOT EDIT!


#ifndef WALKING_CORE_MSGS_MESSAGE_FOOTSTEPS_H
#define WALKING_CORE_MSGS_MESSAGE_FOOTSTEPS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <walking_core_msgs/FootStep.h>

namespace walking_core_msgs
{
template <class ContainerAllocator>
struct FootSteps_
{
  typedef FootSteps_<ContainerAllocator> Type;

  FootSteps_()
    : header()
    , footsteps()  {
    }
  FootSteps_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , footsteps(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::walking_core_msgs::FootStep_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::walking_core_msgs::FootStep_<ContainerAllocator> >> _footsteps_type;
  _footsteps_type footsteps;





  typedef boost::shared_ptr< ::walking_core_msgs::FootSteps_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::walking_core_msgs::FootSteps_<ContainerAllocator> const> ConstPtr;

}; // struct FootSteps_

typedef ::walking_core_msgs::FootSteps_<std::allocator<void> > FootSteps;

typedef boost::shared_ptr< ::walking_core_msgs::FootSteps > FootStepsPtr;
typedef boost::shared_ptr< ::walking_core_msgs::FootSteps const> FootStepsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::walking_core_msgs::FootSteps_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::walking_core_msgs::FootSteps_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::walking_core_msgs::FootSteps_<ContainerAllocator1> & lhs, const ::walking_core_msgs::FootSteps_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.footsteps == rhs.footsteps;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::walking_core_msgs::FootSteps_<ContainerAllocator1> & lhs, const ::walking_core_msgs::FootSteps_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace walking_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::walking_core_msgs::FootSteps_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::walking_core_msgs::FootSteps_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::walking_core_msgs::FootSteps_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "afa335a6af8741fb5411d66b054528e3";
  }

  static const char* value(const ::walking_core_msgs::FootSteps_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xafa335a6af8741fbULL;
  static const uint64_t static_value2 = 0x5411d66b054528e3ULL;
};

template<class ContainerAllocator>
struct DataType< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "walking_core_msgs/FootSteps";
  }

  static const char* value(const ::walking_core_msgs::FootSteps_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"FootStep[] footsteps\n"
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
"MSG: walking_core_msgs/FootStep\n"
"control_core_msgs/Contact contact\n"
"std_msgs/Int64 body_id\n"
"std_msgs/Bool final_step\n"
"std_msgs/Int64 n_step\n"
"================================================================================\n"
"MSG: control_core_msgs/Contact\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Polygon hull\n"
"geometry_msgs/Point offset\n"
"std_msgs/Float64 friction\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Polygon\n"
"#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"================================================================================\n"
"MSG: std_msgs/Float64\n"
"float64 data\n"
"================================================================================\n"
"MSG: std_msgs/Int64\n"
"int64 data\n"
"================================================================================\n"
"MSG: std_msgs/Bool\n"
"bool data\n"
;
  }

  static const char* value(const ::walking_core_msgs::FootSteps_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.footsteps);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FootSteps_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::walking_core_msgs::FootSteps_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::walking_core_msgs::FootSteps_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "footsteps[]" << std::endl;
    for (size_t i = 0; i < v.footsteps.size(); ++i)
    {
      s << indent << "  footsteps[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::walking_core_msgs::FootStep_<ContainerAllocator> >::stream(s, indent + "    ", v.footsteps[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // WALKING_CORE_MSGS_MESSAGE_FOOTSTEPS_H
