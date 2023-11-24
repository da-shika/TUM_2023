#include <ros/ros.h>
#include <skin_visualizer/skin_visualizer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skin_visulizer");
    ROS_INFO_STREAM("starting skin_visulizer...");
    ros::NodeHandle nh("~");

    // wait for skin to be ready
    ros::Duration(2.0).sleep();

    std::vector<std::string> inputs;
    if(argc > 1)
    {
      inputs.assign(argv + 1, argv + argc);
    }
    if(inputs.size() < 1)
    {
      ROS_ERROR("Missing Input arguments, Requires: [target_frame]");
      return -1;
    }
    else if(inputs.size() < 2)
    {
      inputs.push_back("");
    }

    cc::Parameters params;
    const std::string& target_frame = inputs[0];
    const std::string& tf_prefix = inputs[1];
    skin_visualizer::SkinVisualizer visualizer("skin_visulizer", target_frame, tf_prefix);
    if(!visualizer.initRequest(nh, params))
    {
      ROS_ERROR("skin_visulizer main(): Cant initalize visualizer");
      return -1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // loop
    ////////////////////////////////////////////////////////////////////////////

    ros::Rate rate(25);
    ros::Time time;
    ros::Time prev_time = ros::Time::now();
    ros::Duration dt;

    ros::Duration(1.0).sleep();
    visualizer.startRequest(prev_time);
    while(ros::ok())
    {
      time = ros::Time::now();
      dt = time - prev_time;
      prev_time = time;

      visualizer.updateRequest(time, dt);
    
      ros::spinOnce();
      rate.sleep();
    }

    ros::spin();
    return 0;
}