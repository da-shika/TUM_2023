// Generated by gencpp from file control_core_msgs/ComputeControlResponse.msg
// DO NOT EDIT!


#ifndef CONTROL_CORE_MSGS_MESSAGE_COMPUTECONTROLRESPONSE_H
#define CONTROL_CORE_MSGS_MESSAGE_COMPUTECONTROLRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <control_core_msgs/JointState.h>

namespace control_core_msgs
{
template <class ContainerAllocator>
struct ComputeControlResponse_
{
  typedef ComputeControlResponse_<ContainerAllocator> Type;

  ComputeControlResponse_()
    : command()
    , do_stop(0)  {
    }
  ComputeControlResponse_(const ContainerAllocator& _alloc)
    : command(_alloc)
    , do_stop(0)  {
  (void)_alloc;
    }



   typedef  ::control_core_msgs::JointState_<ContainerAllocator>  _command_type;
  _command_type command;

   typedef uint8_t _do_stop_type;
  _do_stop_type do_stop;





  typedef boost::shared_ptr< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ComputeControlResponse_

typedef ::control_core_msgs::ComputeControlResponse_<std::allocator<void> > ComputeControlResponse;

typedef boost::shared_ptr< ::control_core_msgs::ComputeControlResponse > ComputeControlResponsePtr;
typedef boost::shared_ptr< ::control_core_msgs::ComputeControlResponse const> ComputeControlResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator1> & lhs, const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator2> & rhs)
{
  return lhs.command == rhs.command &&
    lhs.do_stop == rhs.do_stop;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator1> & lhs, const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace control_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e1d5e39018c55415d1bd96b8a2c1d90f";
  }

  static const char* value(const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe1d5e39018c55415ULL;
  static const uint64_t static_value2 = 0xd1bd96b8a2c1d90fULL;
};

template<class ContainerAllocator>
struct DataType< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_core_msgs/ComputeControlResponse";
  }

  static const char* value(const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "JointState command  # current robot command\n"
"uint8 do_stop       # client request stop\n"
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

  static const char* value(const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.command);
      stream.next(m.do_stop);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ComputeControlResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_core_msgs::ComputeControlResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_core_msgs::ComputeControlResponse_<ContainerAllocator>& v)
  {
    s << indent << "command: ";
    s << std::endl;
    Printer< ::control_core_msgs::JointState_<ContainerAllocator> >::stream(s, indent + "  ", v.command);
    s << indent << "do_stop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.do_stop);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_CORE_MSGS_MESSAGE_COMPUTECONTROLRESPONSE_H
