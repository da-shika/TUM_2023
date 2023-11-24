#ifndef SKIN_VISUALIZER_UTILITIES_H_
#define SKIN_VISUALIZER_UTILITIES_H_

#include <eigen3/Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>

namespace skin_visualizer
{

  /**
   * @brief Create a cube marker object
   * 
   * @param id 
   * @param frame 
   * @param r 
   * @param g 
   * @param b 
   * @return visualization_msgs::Marker 
   */
  visualization_msgs::Marker create_cube_marker(
    int id, const std::string& frame, double r=0.3, double g=0.3, double b=0.3, double a=1.0)
  {
    visualization_msgs::Marker cube;
    cube.header.frame_id = frame;
    cube.header.stamp = ros::Time();
    cube.ns = "skin_objects";
    cube.id = id;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::ADD;
    cube.pose.position.x = 0;
    cube.pose.position.y = 0;
    cube.pose.position.z = 0;
    cube.pose.orientation.x = 0.0;
    cube.pose.orientation.y = 0.0;
    cube.pose.orientation.z = 0.0;
    cube.pose.orientation.w = 1.0;
    cube.scale.x = 0.1;
    cube.scale.y = 0.1;
    cube.scale.z = 0.1;
    cube.color.a = 0.0;
    cube.color.r = r;
    cube.color.g = g;
    cube.color.b = b;
    cube.color.a = a;
    return cube;
  }

  /**
   * @brief update existing cube dimensions
   * 
   * @param cube 
   * @param pt_min 
   * @param pt_max 
   * @param r 
   * @param g 
   * @param b 
   */
  void update_cube_dimensions(
    visualization_msgs::Marker& cube,
    const Eigen::Vector3d& pt_min, 
    const Eigen::Vector3d& pt_max,
    double r=0.3, double g=0.3, double b=0.3, double a=1.0)
  {
    Eigen::Vector3d c = 0.5*(pt_max + pt_min);
    Eigen::Vector3d s = pt_max - pt_min;

    cube.pose.position.x = c.x();
    cube.pose.position.y = c.y();
    cube.pose.position.z = c.z();
    cube.scale.x = s.x();
    cube.scale.y = s.y();
    cube.scale.z = s.z();
    cube.color.a = 0.6;

    cube.color.r = r;
    cube.color.g = g;
    cube.color.b = b;
    cube.color.a = a;
  }

  

}

#endif