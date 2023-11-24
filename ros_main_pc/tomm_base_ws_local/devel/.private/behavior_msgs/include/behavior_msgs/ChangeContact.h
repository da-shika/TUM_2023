// Generated by gencpp from file behavior_msgs/ChangeContact.msg
// DO NOT EDIT!


#ifndef BEHAVIOR_MSGS_MESSAGE_CHANGECONTACT_H
#define BEHAVIOR_MSGS_MESSAGE_CHANGECONTACT_H

#include <ros/service_traits.h>


#include <behavior_msgs/ChangeContactRequest.h>
#include <behavior_msgs/ChangeContactResponse.h>


namespace behavior_msgs
{

struct ChangeContact
{

typedef ChangeContactRequest Request;
typedef ChangeContactResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ChangeContact
} // namespace behavior_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::behavior_msgs::ChangeContact > {
  static const char* value()
  {
    return "fb67b79c35378211caa96d9ff91d121b";
  }

  static const char* value(const ::behavior_msgs::ChangeContact&) { return value(); }
};

template<>
struct DataType< ::behavior_msgs::ChangeContact > {
  static const char* value()
  {
    return "behavior_msgs/ChangeContact";
  }

  static const char* value(const ::behavior_msgs::ChangeContact&) { return value(); }
};


// service_traits::MD5Sum< ::behavior_msgs::ChangeContactRequest> should match
// service_traits::MD5Sum< ::behavior_msgs::ChangeContact >
template<>
struct MD5Sum< ::behavior_msgs::ChangeContactRequest>
{
  static const char* value()
  {
    return MD5Sum< ::behavior_msgs::ChangeContact >::value();
  }
  static const char* value(const ::behavior_msgs::ChangeContactRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::behavior_msgs::ChangeContactRequest> should match
// service_traits::DataType< ::behavior_msgs::ChangeContact >
template<>
struct DataType< ::behavior_msgs::ChangeContactRequest>
{
  static const char* value()
  {
    return DataType< ::behavior_msgs::ChangeContact >::value();
  }
  static const char* value(const ::behavior_msgs::ChangeContactRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::behavior_msgs::ChangeContactResponse> should match
// service_traits::MD5Sum< ::behavior_msgs::ChangeContact >
template<>
struct MD5Sum< ::behavior_msgs::ChangeContactResponse>
{
  static const char* value()
  {
    return MD5Sum< ::behavior_msgs::ChangeContact >::value();
  }
  static const char* value(const ::behavior_msgs::ChangeContactResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::behavior_msgs::ChangeContactResponse> should match
// service_traits::DataType< ::behavior_msgs::ChangeContact >
template<>
struct DataType< ::behavior_msgs::ChangeContactResponse>
{
  static const char* value()
  {
    return DataType< ::behavior_msgs::ChangeContact >::value();
  }
  static const char* value(const ::behavior_msgs::ChangeContactResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // BEHAVIOR_MSGS_MESSAGE_CHANGECONTACT_H
