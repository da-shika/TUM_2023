#include <joy_target/joy_target.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"joy_target", ros::init_options::AnonymousName);
  joy_target::JoyTarget app("joy_target");

  ros::NodeHandle nh;

  cc::Parameters global_params;
  if(!app.initRequest(nh, global_params))
  {
    ROS_ERROR_STREAM("main(): joy_target failed to init");
    return -1;
  }

  // run planner node
  cc::Scalar rate = 250;
  ros::Rate r(rate);
  ros::Time time, prev_time = ros::Time::now();

  app.startRequest(ros::Time::now());
  while( ros::ok() )
  {
    time = ros::Time::now();
    ros::Duration dt = time - prev_time;
    prev_time = time;

    app.updateRequest(time, dt);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}