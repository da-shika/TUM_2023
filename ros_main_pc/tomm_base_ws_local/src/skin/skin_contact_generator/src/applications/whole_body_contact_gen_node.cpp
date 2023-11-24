#include <ros/ros.h>
#include <skin_contact_generator/whole_body_contact_generator.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skin_contact_generator");
    ROS_INFO_STREAM("starting skin_contact_generator...");
    ros::NodeHandle nh;
    cc::Parameters params;  

    // wait for skin to be ready
    ros::Duration(5.0).sleep();
    skin_contact_generator::WholeBodyContactGenerator generator("skin_contact_generator");
    if(!generator.initRequest(nh, params))
    {
      ROS_ERROR("skin_contact_generator main(): Cant initalize generator");
      return -1;
    }

    ROS_INFO_STREAM("running skin_contact_generator...");
    ////////////////////////////////////////////////////////////////////////////
    // run
    ////////////////////////////////////////////////////////////////////////////
    ros::Rate rate(250);
    ros::Time time;
    ros::Time prev_time = ros::Time::now();
    ros::Duration dt;

    ros::Duration(1.0).sleep();
    generator.startRequest(prev_time);
    while(ros::ok())
    {
      time = ros::Time::now();
      dt = time - prev_time;
      prev_time = time;

      generator.updateRequest(time, dt);

      ros::spinOnce();
      rate.sleep();
    }

    ros::spin();
    return 0;
}