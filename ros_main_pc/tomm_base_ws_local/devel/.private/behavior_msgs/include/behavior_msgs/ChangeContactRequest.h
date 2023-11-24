// Generated by gencpp from file behavior_msgs/ChangeContactRequest.msg
// DO NOT EDIT!


#ifndef BEHAVIOR_MSGS_MESSAGE_CHANGECONTACTREQUEST_H
#define BEHAVIOR_MSGS_MESSAGE_CHANGECONTACTREQUEST_H


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
struct ChangeContactRequest_
{
  typedef ChangeContactRequest_<ContainerAllocator> Type;

  ChangeContactRequest_()
    : activate()
    , deactivate()  {
    }
  ChangeContactRequest_(const ContainerAllocator& _alloc)
    : activate(_alloc)
    , deactivate(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _activate_type;
  _activate_type activate;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _deactivate_type;
  _deactivate_type deactivate;





  typedef boost::shared_ptr< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ChangeContactRequest_

typedef ::behavior_msgs::ChangeContactRequest_<std::allocator<void> > ChangeContactRequest;

typedef boost::shared_ptr< ::behavior_msgs::ChangeContactRequest > ChangeContactRequestPtr;
typedef boost::shared_ptr< ::behavior_msgs::ChangeContactRequest const> ChangeContactRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator1> & lhs, const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator2> & rhs)
{
  return lhs.activate == rhs.activate &&
    lhs.deactivate == rhs.deactivate;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator1> & lhs, const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace behavior_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b51902abff0e958587393c965f5ce9d9";
  }

  static const char* value(const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb51902abff0e9585ULL;
  static const uint64_t static_value2 = 0x87393c965f5ce9d9ULL;
};

template<class ContainerAllocator>
struct DataType< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "behavior_msgs/ChangeContactRequest";
  }

  static const char* value(const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] activate\n"
"string[] deactivate\n"
;
  }

  static const char* value(const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.activate);
      stream.next(m.deactivate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChangeContactRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::behavior_msgs::ChangeContactRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::behavior_msgs::ChangeContactRequest_<ContainerAllocator>& v)
  {
    s << indent << "activate[]" << std::endl;
    for (size_t i = 0; i < v.activate.size(); ++i)
    {
      s << indent << "  activate[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.activate[i]);
    }
    s << indent << "deactivate[]" << std::endl;
    for (size_t i = 0; i < v.deactivate.size(); ++i)
    {
      s << indent << "  deactivate[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.deactivate[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEHAVIOR_MSGS_MESSAGE_CHANGECONTACTREQUEST_H
