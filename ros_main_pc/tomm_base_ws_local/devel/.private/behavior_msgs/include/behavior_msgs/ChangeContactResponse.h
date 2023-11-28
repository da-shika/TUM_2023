// Generated by gencpp from file behavior_msgs/ChangeContactResponse.msg
// DO NOT EDIT!


#ifndef BEHAVIOR_MSGS_MESSAGE_CHANGECONTACTRESPONSE_H
#define BEHAVIOR_MSGS_MESSAGE_CHANGECONTACTRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace behavior_msgs
{
template <class ContainerAllocator>
struct ChangeContactResponse_
{
  typedef ChangeContactResponse_<ContainerAllocator> Type;

  ChangeContactResponse_()
    : ok(false)  {
    }
  ChangeContactResponse_(const ContainerAllocator& _alloc)
    : ok(false)  {
  (void)_alloc;
    }



   typedef uint8_t _ok_type;
  _ok_type ok;





  typedef boost::shared_ptr< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ChangeContactResponse_

typedef ::behavior_msgs::ChangeContactResponse_<std::allocator<void> > ChangeContactResponse;

typedef boost::shared_ptr< ::behavior_msgs::ChangeContactResponse > ChangeContactResponsePtr;
typedef boost::shared_ptr< ::behavior_msgs::ChangeContactResponse const> ChangeContactResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator1> & lhs, const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator2> & rhs)
{
  return lhs.ok == rhs.ok;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator1> & lhs, const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace behavior_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f6da3883749771fac40d6deb24a8c02";
  }

  static const char* value(const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f6da3883749771fULL;
  static const uint64_t static_value2 = 0xac40d6deb24a8c02ULL;
};

template<class ContainerAllocator>
struct DataType< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "behavior_msgs/ChangeContactResponse";
  }

  static const char* value(const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool ok\n"
;
  }

  static const char* value(const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ok);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChangeContactResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::behavior_msgs::ChangeContactResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::behavior_msgs::ChangeContactResponse_<ContainerAllocator>& v)
  {
    s << indent << "ok: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ok);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEHAVIOR_MSGS_MESSAGE_CHANGECONTACTRESPONSE_H