// Generated by gencpp from file behavior_msgs/Timing.msg
// DO NOT EDIT!


#ifndef BEHAVIOR_MSGS_MESSAGE_TIMING_H
#define BEHAVIOR_MSGS_MESSAGE_TIMING_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64.h>

namespace behavior_msgs
{
template <class ContainerAllocator>
struct Timing_
{
  typedef Timing_<ContainerAllocator> Type;

  Timing_()
    : control_loop_dur()
    , solver_dur()
    , geometry_dur()  {
    }
  Timing_(const ContainerAllocator& _alloc)
    : control_loop_dur(_alloc)
    , solver_dur(_alloc)
    , geometry_dur(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Int64_<ContainerAllocator>  _control_loop_dur_type;
  _control_loop_dur_type control_loop_dur;

   typedef  ::std_msgs::Int64_<ContainerAllocator>  _solver_dur_type;
  _solver_dur_type solver_dur;

   typedef  ::std_msgs::Int64_<ContainerAllocator>  _geometry_dur_type;
  _geometry_dur_type geometry_dur;





  typedef boost::shared_ptr< ::behavior_msgs::Timing_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::behavior_msgs::Timing_<ContainerAllocator> const> ConstPtr;

}; // struct Timing_

typedef ::behavior_msgs::Timing_<std::allocator<void> > Timing;

typedef boost::shared_ptr< ::behavior_msgs::Timing > TimingPtr;
typedef boost::shared_ptr< ::behavior_msgs::Timing const> TimingConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::behavior_msgs::Timing_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::behavior_msgs::Timing_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::behavior_msgs::Timing_<ContainerAllocator1> & lhs, const ::behavior_msgs::Timing_<ContainerAllocator2> & rhs)
{
  return lhs.control_loop_dur == rhs.control_loop_dur &&
    lhs.solver_dur == rhs.solver_dur &&
    lhs.geometry_dur == rhs.geometry_dur;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::behavior_msgs::Timing_<ContainerAllocator1> & lhs, const ::behavior_msgs::Timing_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace behavior_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::Timing_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::Timing_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::Timing_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::Timing_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::Timing_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::Timing_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::behavior_msgs::Timing_<ContainerAllocator> >
{
  static const char* value()
  {
    return "860a4c96fe2b2a5b2000e2a96ca33c2c";
  }

  static const char* value(const ::behavior_msgs::Timing_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x860a4c96fe2b2a5bULL;
  static const uint64_t static_value2 = 0x2000e2a96ca33c2cULL;
};

template<class ContainerAllocator>
struct DataType< ::behavior_msgs::Timing_<ContainerAllocator> >
{
  static const char* value()
  {
    return "behavior_msgs/Timing";
  }

  static const char* value(const ::behavior_msgs::Timing_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::behavior_msgs::Timing_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Int64 control_loop_dur\n"
"std_msgs/Int64 solver_dur\n"
"std_msgs/Int64 geometry_dur\n"
"================================================================================\n"
"MSG: std_msgs/Int64\n"
"int64 data\n"
;
  }

  static const char* value(const ::behavior_msgs::Timing_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::behavior_msgs::Timing_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.control_loop_dur);
      stream.next(m.solver_dur);
      stream.next(m.geometry_dur);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Timing_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::behavior_msgs::Timing_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::behavior_msgs::Timing_<ContainerAllocator>& v)
  {
    s << indent << "control_loop_dur: ";
    s << std::endl;
    Printer< ::std_msgs::Int64_<ContainerAllocator> >::stream(s, indent + "  ", v.control_loop_dur);
    s << indent << "solver_dur: ";
    s << std::endl;
    Printer< ::std_msgs::Int64_<ContainerAllocator> >::stream(s, indent + "  ", v.solver_dur);
    s << indent << "geometry_dur: ";
    s << std::endl;
    Printer< ::std_msgs::Int64_<ContainerAllocator> >::stream(s, indent + "  ", v.geometry_dur);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEHAVIOR_MSGS_MESSAGE_TIMING_H
