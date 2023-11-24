// Generated by gencpp from file control_core_msgs/SetCartesianStateGoal.msg
// DO NOT EDIT!


#ifndef CONTROL_CORE_MSGS_MESSAGE_SETCARTESIANSTATEGOAL_H
#define CONTROL_CORE_MSGS_MESSAGE_SETCARTESIANSTATEGOAL_H

#include <ros/service_traits.h>


#include <control_core_msgs/SetCartesianStateGoalRequest.h>
#include <control_core_msgs/SetCartesianStateGoalResponse.h>


namespace control_core_msgs
{

struct SetCartesianStateGoal
{

typedef SetCartesianStateGoalRequest Request;
typedef SetCartesianStateGoalResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCartesianStateGoal
} // namespace control_core_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::control_core_msgs::SetCartesianStateGoal > {
  static const char* value()
  {
    return "2f3406a57b13431c34e22c35d33bdee4";
  }

  static const char* value(const ::control_core_msgs::SetCartesianStateGoal&) { return value(); }
};

template<>
struct DataType< ::control_core_msgs::SetCartesianStateGoal > {
  static const char* value()
  {
    return "control_core_msgs/SetCartesianStateGoal";
  }

  static const char* value(const ::control_core_msgs::SetCartesianStateGoal&) { return value(); }
};


// service_traits::MD5Sum< ::control_core_msgs::SetCartesianStateGoalRequest> should match
// service_traits::MD5Sum< ::control_core_msgs::SetCartesianStateGoal >
template<>
struct MD5Sum< ::control_core_msgs::SetCartesianStateGoalRequest>
{
  static const char* value()
  {
    return MD5Sum< ::control_core_msgs::SetCartesianStateGoal >::value();
  }
  static const char* value(const ::control_core_msgs::SetCartesianStateGoalRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::control_core_msgs::SetCartesianStateGoalRequest> should match
// service_traits::DataType< ::control_core_msgs::SetCartesianStateGoal >
template<>
struct DataType< ::control_core_msgs::SetCartesianStateGoalRequest>
{
  static const char* value()
  {
    return DataType< ::control_core_msgs::SetCartesianStateGoal >::value();
  }
  static const char* value(const ::control_core_msgs::SetCartesianStateGoalRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::control_core_msgs::SetCartesianStateGoalResponse> should match
// service_traits::MD5Sum< ::control_core_msgs::SetCartesianStateGoal >
template<>
struct MD5Sum< ::control_core_msgs::SetCartesianStateGoalResponse>
{
  static const char* value()
  {
    return MD5Sum< ::control_core_msgs::SetCartesianStateGoal >::value();
  }
  static const char* value(const ::control_core_msgs::SetCartesianStateGoalResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::control_core_msgs::SetCartesianStateGoalResponse> should match
// service_traits::DataType< ::control_core_msgs::SetCartesianStateGoal >
template<>
struct DataType< ::control_core_msgs::SetCartesianStateGoalResponse>
{
  static const char* value()
  {
    return DataType< ::control_core_msgs::SetCartesianStateGoal >::value();
  }
  static const char* value(const ::control_core_msgs::SetCartesianStateGoalResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CONTROL_CORE_MSGS_MESSAGE_SETCARTESIANSTATEGOAL_H
