#include <control_core/statemachine/tiny_statemachine.h>
#include <ros/ros.h>

enum StateId {S1, S2, S3, N_STATES};
enum EventId {E1, E2, E3, JUMP, TICK, N_EVENTS};

class StateMachineImpl : public cc::StateMachine<StateMachineImpl, StateId, EventId>
{
public:
  typedef cc::StateMachine<StateMachineImpl, StateId, EventId> Base;

public:
  StateMachineImpl() : Base(S1, N_STATES, N_EVENTS)
  {
    Base::add({
      {S1, S2, E1, &StateMachineImpl::transitS1},
      {S1, S1, TICK, &StateMachineImpl::processS1},
      {S2, S3, E2, &StateMachineImpl::transitS2},
      {S2, S2, TICK, &StateMachineImpl::processS2},
      {S3, S3, E3, &StateMachineImpl::transitS3},
      {S3, S3, TICK, &StateMachineImpl::processS3}});
  }
  virtual ~StateMachineImpl() {}

  void transitS1() { ROS_INFO_STREAM("S1->S2"); }
  void transitS2() { ROS_INFO_STREAM("S2->S3"); }
  void transitS3() { ROS_INFO_STREAM("S3->End"); }

  void processS1() { ROS_INFO_STREAM("process S1"); }
  void processS2() { ROS_INFO_STREAM("process S2"); Base::handle(E2); }
  void processS3() { ROS_INFO_STREAM("process S3"); }
};


int main(int argc, char **argv)
{  
  ros::init(argc, argv, "test_statemachine");
  ros::NodeHandle nh;

  StateMachineImpl sm;

  ros::Time time = ros::Time::now();
  ros::Duration period(1.0);
  for(int i = 0; i < 15; ++i)
  {
    ROS_WARN_STREAM("STEP i=" << i);

    if(i == 5)
      sm.handle(E1);
    sm.handle(TICK);

    period.sleep();
  }
}