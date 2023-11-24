#include <control_core/statemachine/statemachine_base.h>


////////////////////////////////////////////////////////////////////////////////
// define event
////////////////////////////////////////////////////////////////////////////////
struct Event { int id; };

////////////////////////////////////////////////////////////////////////////////
// define context
////////////////////////////////////////////////////////////////////////////////
struct Context { 
  std::string context; 
  Context() : context{"default"} {}
};

////////////////////////////////////////////////////////////////////////////////
// define state ids: Note: reserve 0 for None state
////////////////////////////////////////////////////////////////////////////////
enum StateId { NONE, S1, S2, S3, S4 };

////////////////////////////////////////////////////////////////////////////////
// define statemachine
////////////////////////////////////////////////////////////////////////////////
class StateMachine : public cc::StateMachineBase<StateMachine, Context, Event, StateId>
{
public: 
  typedef cc::StateMachineBase<StateMachine, Context, Event, StateId> Base;

public:
  StateMachine(const std::string& name) : Base(name, true) {}
  virtual ~StateMachine() {}
};

////////////////////////////////////////////////////////////////////////////////
// define state
////////////////////////////////////////////////////////////////////////////////
class PrintState : public cc::StateBase<StateMachine>
{
public:
  typedef cc::StateBase<StateMachine> Base;

private:
  StateId next_;

public:
  PrintState(const std::string& name, const StateId& id, const StateId& next) : 
    Base(name, id), next_(next) {}
  virtual ~PrintState() {}

protected:
  virtual bool init(ros::NodeHandle& nh, cc::Parameters& parameter) override {
    ROS_INFO_STREAM(Base::name() << " : init()");
    return true;
  }
  virtual void start(const ros::Time& time, const StateId& prev) override {
    ROS_INFO_STREAM(Base::name() << " : start()");
  }
  virtual bool update(const ros::Time &time, const ros::Duration &period) override {
    ROS_INFO_STREAM(Base::name() << " : update()");
    return Base::change(time, next_);
  }
  virtual void stop(const ros::Time& time, const StateId& next) override {
    ROS_INFO_STREAM(Base::name() << " : stop()");
  }
};

// test statemachine
int main(int argc, char **argv)
{  
  ros::init(argc, argv, "test_statemachine");
  ros::NodeHandle nh;
  cc::Parameters parameter;

  // s1->s2->s3->s4->s1->...
  StateMachine sm("test_sm");
  sm.add(std::make_shared<PrintState>("s1", S1, S2));
  sm.add(std::make_shared<PrintState>("s2", S2, S3));
  sm.add(std::make_shared<PrintState>("s3", S3, S4));
  sm.add(std::make_shared<PrintState>("s4", S4, S1));

  sm.initRequest(nh, parameter);

  ros::Time time = ros::Time::now();
  ros::Duration period(1.0);
  sm.startRequest(ros::Time::now());

  for(int i = 0; i < 15; ++i)
  {
    ROS_WARN_STREAM("STEP i=" << i);
    sm.updateRequest(time, period);
    period.sleep();
  }
  return 0;
}