// Generated by gencpp from file ics_tsid_task_msgs/SkinDistanceConstraint.msg
// DO NOT EDIT!


#ifndef ICS_TSID_TASK_MSGS_MESSAGE_SKINDISTANCECONSTRAINT_H
#define ICS_TSID_TASK_MSGS_MESSAGE_SKINDISTANCECONSTRAINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <control_core_msgs/Vector.h>
#include <control_core_msgs/Vector.h>
#include <control_core_msgs/Vector.h>
#include <control_core_msgs/Vector.h>
#include <control_core_msgs/Vector.h>
#include <std_msgs/Float64.h>
#include <control_core_msgs/Vector.h>

namespace ics_tsid_task_msgs
{
template <class ContainerAllocator>
struct SkinDistanceConstraint_
{
  typedef SkinDistanceConstraint_<ContainerAllocator> Type;

  SkinDistanceConstraint_()
    : dur()
    , num_active_ieq()
    , num_violated_ieq()
    , conflict()
    , max_force()
    , min_distance()
    , max_proximity()
    , distances()
    , cell_nums()
    , vel_limits()
    , vel_cmds()
    , acc_limits()
    , weight()
    , a_relaxed()  {
    }
  SkinDistanceConstraint_(const ContainerAllocator& _alloc)
    : dur(_alloc)
    , num_active_ieq(_alloc)
    , num_violated_ieq(_alloc)
    , conflict(_alloc)
    , max_force(_alloc)
    , min_distance(_alloc)
    , max_proximity(_alloc)
    , distances(_alloc)
    , cell_nums(_alloc)
    , vel_limits(_alloc)
    , vel_cmds(_alloc)
    , acc_limits(_alloc)
    , weight(_alloc)
    , a_relaxed(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Int64_<ContainerAllocator>  _dur_type;
  _dur_type dur;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _num_active_ieq_type;
  _num_active_ieq_type num_active_ieq;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _num_violated_ieq_type;
  _num_violated_ieq_type num_violated_ieq;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _conflict_type;
  _conflict_type conflict;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _max_force_type;
  _max_force_type max_force;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _min_distance_type;
  _min_distance_type min_distance;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _max_proximity_type;
  _max_proximity_type max_proximity;

   typedef  ::control_core_msgs::Vector_<ContainerAllocator>  _distances_type;
  _distances_type distances;

   typedef  ::control_core_msgs::Vector_<ContainerAllocator>  _cell_nums_type;
  _cell_nums_type cell_nums;

   typedef  ::control_core_msgs::Vector_<ContainerAllocator>  _vel_limits_type;
  _vel_limits_type vel_limits;

   typedef  ::control_core_msgs::Vector_<ContainerAllocator>  _vel_cmds_type;
  _vel_cmds_type vel_cmds;

   typedef  ::control_core_msgs::Vector_<ContainerAllocator>  _acc_limits_type;
  _acc_limits_type acc_limits;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _weight_type;
  _weight_type weight;

   typedef  ::control_core_msgs::Vector_<ContainerAllocator>  _a_relaxed_type;
  _a_relaxed_type a_relaxed;





  typedef boost::shared_ptr< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> const> ConstPtr;

}; // struct SkinDistanceConstraint_

typedef ::ics_tsid_task_msgs::SkinDistanceConstraint_<std::allocator<void> > SkinDistanceConstraint;

typedef boost::shared_ptr< ::ics_tsid_task_msgs::SkinDistanceConstraint > SkinDistanceConstraintPtr;
typedef boost::shared_ptr< ::ics_tsid_task_msgs::SkinDistanceConstraint const> SkinDistanceConstraintConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator1> & lhs, const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator2> & rhs)
{
  return lhs.dur == rhs.dur &&
    lhs.num_active_ieq == rhs.num_active_ieq &&
    lhs.num_violated_ieq == rhs.num_violated_ieq &&
    lhs.conflict == rhs.conflict &&
    lhs.max_force == rhs.max_force &&
    lhs.min_distance == rhs.min_distance &&
    lhs.max_proximity == rhs.max_proximity &&
    lhs.distances == rhs.distances &&
    lhs.cell_nums == rhs.cell_nums &&
    lhs.vel_limits == rhs.vel_limits &&
    lhs.vel_cmds == rhs.vel_cmds &&
    lhs.acc_limits == rhs.acc_limits &&
    lhs.weight == rhs.weight &&
    lhs.a_relaxed == rhs.a_relaxed;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator1> & lhs, const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ics_tsid_task_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f1adb03c236ae24d7d056bb4d4bea499";
  }

  static const char* value(const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf1adb03c236ae24dULL;
  static const uint64_t static_value2 = 0x7d056bb4d4bea499ULL;
};

template<class ContainerAllocator>
struct DataType< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ics_tsid_task_msgs/SkinDistanceConstraint";
  }

  static const char* value(const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Int64 dur\n"
"std_msgs/Int32 num_active_ieq\n"
"std_msgs/Int32 num_violated_ieq\n"
"std_msgs/Int32 conflict\n"
"\n"
"std_msgs/Float64 max_force\n"
"std_msgs/Float64 min_distance\n"
"std_msgs/Float64 max_proximity\n"
"\n"
"control_core_msgs/Vector distances\n"
"control_core_msgs/Vector cell_nums\n"
"control_core_msgs/Vector vel_limits\n"
"control_core_msgs/Vector vel_cmds\n"
"control_core_msgs/Vector acc_limits\n"
"\n"
"std_msgs/Float64 weight\n"
"control_core_msgs/Vector a_relaxed\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Int64\n"
"int64 data\n"
"================================================================================\n"
"MSG: std_msgs/Int32\n"
"int32 data\n"
"================================================================================\n"
"MSG: std_msgs/Float64\n"
"float64 data\n"
"================================================================================\n"
"MSG: control_core_msgs/Vector\n"
"float64[] data\n"
;
  }

  static const char* value(const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dur);
      stream.next(m.num_active_ieq);
      stream.next(m.num_violated_ieq);
      stream.next(m.conflict);
      stream.next(m.max_force);
      stream.next(m.min_distance);
      stream.next(m.max_proximity);
      stream.next(m.distances);
      stream.next(m.cell_nums);
      stream.next(m.vel_limits);
      stream.next(m.vel_cmds);
      stream.next(m.acc_limits);
      stream.next(m.weight);
      stream.next(m.a_relaxed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SkinDistanceConstraint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ics_tsid_task_msgs::SkinDistanceConstraint_<ContainerAllocator>& v)
  {
    s << indent << "dur: ";
    s << std::endl;
    Printer< ::std_msgs::Int64_<ContainerAllocator> >::stream(s, indent + "  ", v.dur);
    s << indent << "num_active_ieq: ";
    s << std::endl;
    Printer< ::std_msgs::Int32_<ContainerAllocator> >::stream(s, indent + "  ", v.num_active_ieq);
    s << indent << "num_violated_ieq: ";
    s << std::endl;
    Printer< ::std_msgs::Int32_<ContainerAllocator> >::stream(s, indent + "  ", v.num_violated_ieq);
    s << indent << "conflict: ";
    s << std::endl;
    Printer< ::std_msgs::Int32_<ContainerAllocator> >::stream(s, indent + "  ", v.conflict);
    s << indent << "max_force: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.max_force);
    s << indent << "min_distance: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.min_distance);
    s << indent << "max_proximity: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.max_proximity);
    s << indent << "distances: ";
    s << std::endl;
    Printer< ::control_core_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.distances);
    s << indent << "cell_nums: ";
    s << std::endl;
    Printer< ::control_core_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.cell_nums);
    s << indent << "vel_limits: ";
    s << std::endl;
    Printer< ::control_core_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.vel_limits);
    s << indent << "vel_cmds: ";
    s << std::endl;
    Printer< ::control_core_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.vel_cmds);
    s << indent << "acc_limits: ";
    s << std::endl;
    Printer< ::control_core_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.acc_limits);
    s << indent << "weight: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.weight);
    s << indent << "a_relaxed: ";
    s << std::endl;
    Printer< ::control_core_msgs::Vector_<ContainerAllocator> >::stream(s, indent + "  ", v.a_relaxed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ICS_TSID_TASK_MSGS_MESSAGE_SKINDISTANCECONSTRAINT_H