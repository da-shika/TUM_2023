#include <joy_target/marker_target.h>

int main(int argc, char **argv)
{
  // wait for tf tree to come up
  ros::init(argc,argv,"marker_target", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ros::Duration(2.0).sleep();

  cc::Parameters global_params;
  joy_target::MarkerTarget app("marker_target");
  if(!app.initRequest(nh, global_params))
  {
    ROS_ERROR_STREAM("main(): marker_target failed to init");
    return -1;
  }

  // run planner node
  cc::Scalar rate = 100;
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