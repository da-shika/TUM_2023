#include <ros/ros.h>
#include <skin_contact_generator/patch_contact_generator.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "patch_contact_generator", ros::init_options::AnonymousName);
    ROS_INFO_STREAM("starting patch_contact_generator...");
    ros::NodeHandle nh;
    cc::Parameters params;  

    // wait for skin to be ready
    ros::Duration(2.0).sleep();

    ////////////////////////////////////////////////////////////////////////////
    // read input and init
    ////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> inputs;
    if(argc > 1)
    {
      inputs.assign(argv + 1, argv + argc);
    }
    if(inputs.size() < 2)
    {
      ROS_ERROR("Missing Input arguments, Requires two: [name, patch_name]");
      for(const auto& input : inputs)
        ROS_WARN_STREAM("Got: " << input);
      return -1;
    }
    const auto& name = inputs[0];
    const auto& patch_name = inputs[1];
    skin_contact_generator::PatchContactGenerator generator(name, patch_name);
    if(!generator.initRequest(nh, params))
    {
      ROS_ERROR("patch_contact_generator main(): Cant initalize generator");
      return -1;
    }

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