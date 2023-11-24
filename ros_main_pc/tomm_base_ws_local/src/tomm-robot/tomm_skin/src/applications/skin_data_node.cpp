#include <ros/ros.h>
#include <tum_ics_skin_descr/Patch/TfMarkerDataPatches.h>
#include <QApplication>
#include <QFileInfo>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <stdio.h>
#include <stdlib.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <control_core/types.h>
#include <control_core/ros/ros.h>

using namespace tum_ics_skin_descr;
using namespace Eigen;

bool initalizeSkinPatch(const std::string &arm_side,
                        const std::string &config,
                        tum_ics_skin_descr::Patch::TfMarkerDataPatch &patch)
{
  std::string err_str;

  QString skin_config_file = QString::fromStdString(config);
  if (!patch.load(skin_config_file))
  {
    ROS_ERROR_STREAM(arm_side << ": TommSkin::config(): error loading skin config");
    return false;
  }
  if (patch.isUndefined())
  {
    ROS_ERROR_STREAM(arm_side << ": TommSkin::config(): skin patch is undefined");
    return false;
  }
  patch.createDataConnection();
  patch.enableDataConnection();

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "skin_control_node",
            ros::init_options::AnonymousName);
  ros::NodeHandle n;

  ros::Rate r(100);

  ros::Publisher force_pub = n.advertise<std_msgs::Float64MultiArray>("force_arr", 1000);
  ros::Publisher prox_pub = n.advertise<std_msgs::Float64MultiArray>("prox_arr", 1000);
  ros::Publisher temp_pub = n.advertise<std_msgs::Float64MultiArray>("temp_arr", 1000);

  std_msgs::Float64MultiArray force_arr;
  std_msgs::Float64MultiArray prox_arr;
  std_msgs::Float64MultiArray temp_arr;

  // load the config files
  std::string arm_side = "left";
  std::string lower_arm_config_file, upper_arm_config_file;

  // if (!cc::load("~" + arm_side + "_lower_arm_config_file", lower_arm_config_file))
  // {
  //   ROS_ERROR_STREAM(arm_side << ": TommSkin::config(): error reading skin config:" << lower_arm_config_file);
  //   return -1;
  // }
  if (!cc::load("~" + arm_side + "_upper_arm_config_file", upper_arm_config_file))
  {
    ROS_ERROR_STREAM(arm_side << ": TommSkin::config(): error reading skin config:" << upper_arm_config_file);
    return -1;
  }


  // initialize skin patches
  tum_ics_skin_descr::Patch::TfMarkerDataPatch patch_uArm, patch_lArm;
  if (!initalizeSkinPatch(arm_side, upper_arm_config_file, patch_uArm))
  {
    ROS_ERROR_STREAM(arm_side << ": TommSkin::config(): error init skin_patch: " << upper_arm_config_file);
    return -1;
  }
  // if (!initalizeSkinPatch(arm_side, lower_arm_config_file, patch_lArm))
  // {
  //   ROS_ERROR_STREAM(arm_side << ": TommSkin::config(): error init skin_patch: " << lower_arm_config_file);
  //   return -1;
  // }

  while (ros::ok())
  {
    // clear previous msgs
    force_arr.data.clear();
    prox_arr.data.clear();
    temp_arr.data.clear();

    QVector<Skin::Cell::Data> data = patch_uArm.data();
    for (int j = 1; j < data.size(); j += 1)
    { // ADAPT THIS TO DIFFERENT SKIN CELL ARRAY (curr: 16 cells >> how many does the robot have?)
      for (int i = 0; i < 3; i++)
      { // force
        force_arr.data.push_back(data[j].force[i]);
      }
      prox_arr.data.push_back(data[j].prox[0]); // prox
      temp_arr.data.push_back(data[j].temp[0]); // temp
      force_pub.publish(force_arr);
      prox_pub.publish(prox_arr);
      temp_pub.publish(temp_arr);
    }

    ros::spinOnce();
    r.sleep();
  }

  qDebug("exit");

  return 0;
}