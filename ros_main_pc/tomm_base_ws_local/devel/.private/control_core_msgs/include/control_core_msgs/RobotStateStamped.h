// Generated by gencpp from file control_core_msgs/RobotStateStamped.msg
// DO NOT EDIT!


#ifndef CONTROL_CORE_MSGS_MESSAGE_ROBOTSTATESTAMPED_H
#define CONTROL_CORE_MSGS_MESSAGE_ROBOTSTATESTAMPED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <control_core_msgs/RobotState.h>

namespace control_core_msgs
{
template <class ContainerAllocator>
struct RobotStateStamped_
{
  typedef RobotStateStamped_<ContainerAllocator> Type;

  RobotStateStamped_()
    : header()
    , state()  {
    }
  RobotStateStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::control_core_msgs::RobotState_<ContainerAllocator>  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> const> ConstPtr;

}; // struct RobotStateStamped_

typedef ::control_core_msgs::RobotStateStamped_<std::allocator<void> > RobotStateStamped;

typedef boost::shared_ptr< ::control_core_msgs::RobotStateStamped > RobotStateStampedPtr;
typedef boost::shared_ptr< ::control_core_msgs::RobotStateStamped const> RobotStateStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_core_msgs::RobotStateStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::control_core_msgs::RobotStateStamped_<ContainerAllocator1> & lhs, const ::control_core_msgs::RobotStateStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.state == rhs.state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::control_core_msgs::RobotStateStamped_<ContainerAllocator1> & lhs, const ::control_core_msgs::RobotStateStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace control_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "63f336f16841958ab3b1e1ed25060df5";
  }

  static const char* value(const ::control_core_msgs::RobotStateStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x63f336f16841958aULL;
  static const uint64_t static_value2 = 0xb3b1e1ed25060df5ULL;
};

template<class ContainerAllocator>
struct DataType< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_core_msgs/RobotStateStamped";
  }

  static const char* value(const ::control_core_msgs::RobotStateStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"RobotState state\n"
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
"MSG: control_core_msgs/RobotState\n"
"JointState joints\n"
"CartesianState floating_base\n"
"sensor_msgs/Imu imu\n"
"geometry_msgs/WrenchStamped[] ft_sensors\n"
"SkinPatch[] patches\n"
"================================================================================\n"
"MSG: control_core_msgs/JointState\n"
"Vector position\n"
"Vector velocity\n"
"Vector acceleration\n"
"================================================================================\n"
"MSG: control_core_msgs/Vector\n"
"float64[] data\n"
"================================================================================\n"
"MSG: control_core_msgs/CartesianState\n"
"geometry_msgs/Pose position\n"
"geometry_msgs/Twist velocity\n"
"geometry_msgs/Accel acceleration\n"
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
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
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
"================================================================================\n"
"MSG: geometry_msgs/Accel\n"
"# This expresses acceleration in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Imu\n"
"# This is a message to hold data from an IMU (Inertial Measurement Unit)\n"
"#\n"
"# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec\n"
"#\n"
"# If the covariance of the measurement is known, it should be filled in (if all you know is the \n"
"# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)\n"
"# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the\n"
"# data a covariance will have to be assumed or gotten from some other source\n"
"#\n"
"# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation \n"
"# estimate), please set element 0 of the associated covariance matrix to -1\n"
"# If you are interpreting this message, please check for a value of -1 in the first element of each \n"
"# covariance matrix, and disregard the associated estimate.\n"
"\n"
"Header header\n"
"\n"
"geometry_msgs/Quaternion orientation\n"
"float64[9] orientation_covariance # Row major about x, y, z axes\n"
"\n"
"geometry_msgs/Vector3 angular_velocity\n"
"float64[9] angular_velocity_covariance # Row major about x, y, z axes\n"
"\n"
"geometry_msgs/Vector3 linear_acceleration\n"
"float64[9] linear_acceleration_covariance # Row major x, y z \n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/WrenchStamped\n"
"# A wrench with reference coordinate frame and timestamp\n"
"Header header\n"
"Wrench wrench\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Wrench\n"
"# This represents force in free space, separated into\n"
"# its linear and angular parts.\n"
"Vector3  force\n"
"Vector3  torque\n"
"\n"
"================================================================================\n"
"MSG: control_core_msgs/SkinPatch\n"
"std_msgs/Header header\n"
"geometry_msgs/Pose pose\n"
"SkinModality force\n"
"SkinModality proximity\n"
"std_msgs/Float64 min_dist\n"
"std_msgs/Float64 max_dist\n"
"================================================================================\n"
"MSG: control_core_msgs/SkinModality\n"
"std_msgs/Float64 min\n"
"std_msgs/Float64 max\n"
"std_msgs/Float64 area\n"
"geometry_msgs/Point cop\n"
"geometry_msgs/Wrench wrench\n"
"geometry_msgs/Polygon hull\n"
"================================================================================\n"
"MSG: std_msgs/Float64\n"
"float64 data\n"
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
;
  }

  static const char* value(const ::control_core_msgs::RobotStateStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotStateStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_core_msgs::RobotStateStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_core_msgs::RobotStateStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    s << std::endl;
    Printer< ::control_core_msgs::RobotState_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_CORE_MSGS_MESSAGE_ROBOTSTATESTAMPED_H
