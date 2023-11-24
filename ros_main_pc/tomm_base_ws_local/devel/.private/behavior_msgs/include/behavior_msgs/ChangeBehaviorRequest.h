// Generated by gencpp from file behavior_msgs/ChangeBehaviorRequest.msg
// DO NOT EDIT!


#ifndef BEHAVIOR_MSGS_MESSAGE_CHANGEBEHAVIORREQUEST_H
#define BEHAVIOR_MSGS_MESSAGE_CHANGEBEHAVIORREQUEST_H


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
struct ChangeBehaviorRequest_
{
  typedef ChangeBehaviorRequest_<ContainerAllocator> Type;

  ChangeBehaviorRequest_()
    : start_behaviors()
    , stop_behaviors()  {
    }
  ChangeBehaviorRequest_(const ContainerAllocator& _alloc)
    : start_behaviors(_alloc)
    , stop_behaviors(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _start_behaviors_type;
  _start_behaviors_type start_behaviors;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _stop_behaviors_type;
  _stop_behaviors_type stop_behaviors;





  typedef boost::shared_ptr< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ChangeBehaviorRequest_

typedef ::behavior_msgs::ChangeBehaviorRequest_<std::allocator<void> > ChangeBehaviorRequest;

typedef boost::shared_ptr< ::behavior_msgs::ChangeBehaviorRequest > ChangeBehaviorRequestPtr;
typedef boost::shared_ptr< ::behavior_msgs::ChangeBehaviorRequest const> ChangeBehaviorRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator1> & lhs, const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator2> & rhs)
{
  return lhs.start_behaviors == rhs.start_behaviors &&
    lhs.stop_behaviors == rhs.stop_behaviors;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator1> & lhs, const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace behavior_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "43b5cecd3c7bffade9aa666791c6249f";
  }

  static const char* value(const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x43b5cecd3c7bffadULL;
  static const uint64_t static_value2 = 0xe9aa666791c6249fULL;
};

template<class ContainerAllocator>
struct DataType< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "behavior_msgs/ChangeBehaviorRequest";
  }

  static const char* value(const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] start_behaviors\n"
"string[] stop_behaviors\n"
;
  }

  static const char* value(const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.start_behaviors);
      stream.next(m.stop_behaviors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChangeBehaviorRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::behavior_msgs::ChangeBehaviorRequest_<ContainerAllocator>& v)
  {
    s << indent << "start_behaviors[]" << std::endl;
    for (size_t i = 0; i < v.start_behaviors.size(); ++i)
    {
      s << indent << "  start_behaviors[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.start_behaviors[i]);
    }
    s << indent << "stop_behaviors[]" << std::endl;
    for (size_t i = 0; i < v.stop_behaviors.size(); ++i)
    {
      s << indent << "  stop_behaviors[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.stop_behaviors[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEHAVIOR_MSGS_MESSAGE_CHANGEBEHAVIORREQUEST_H
