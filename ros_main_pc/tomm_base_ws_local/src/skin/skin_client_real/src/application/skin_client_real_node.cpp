#include <skin_client_real/skin_client_real.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skin_client_real");
    ROS_INFO_STREAM("starting skin_client_real...");
    ros::NodeHandle nh("/");

    std::vector<std::string> inputs;
    if(argc > 1)
    {
      inputs.assign(argv + 1, argv + argc);
    }
    if(inputs.size() < 1)
    {
      ROS_ERROR("Missing Input arguments, Requires: [patch_config_1, patch_config_2,...]");
      return -1;
    }

    skin_client_real::SkinClientReal client("skin_client_real", inputs);
    
    cc::Parameters global_params;
    if(!client.initRequest(nh, global_params))
    {
      ROS_ERROR("skin_client_real main(): Cant initalize SkinDataClient");
      return -1;
    }

    // wait for connections to setup
    ros::Duration(0.5).sleep();

    // wait for data to arrive, start clients
    client.startRequest(ros::Time::now());
    if(!client.isValid())
    {
      ROS_ERROR("skin_client_real main(): Failed to start SkinDataClient");
      return -1;
    }
    ROS_INFO("skin_client_real start...");
    
    ros::Rate rate(250);
    ros::Time time, prev_time;
    ros::Duration dt;
    while(ros::ok())
    {
      time = ros::Time::now(); 
      dt = time - prev_time;
      prev_time = time;

      client.update(time, dt);

      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}