/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */


#ifndef CONTROL_CORE_MARKER_H
#define CONTROL_CORE_MARKER_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace cc
{

  /**
   * @brief Create a mesh marker
   * Note: the color is only works with .dae files
   * 
   * @param id 
   * @param r 
   * @param g 
   * @param b 
   * @return visualization_msgs::Marker 
   */
  inline visualization_msgs::Marker create_mesh_marker(
    const std::string& ns, 
    int id, 
    const std::string& mesh_file,
    double scale=1.0,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.mesh_resource = mesh_file;
    marker.mesh_use_embedded_materials = true;
    return marker;
  }

  inline visualization_msgs::Marker create_mesh_marker(
    const std::string& ns, 
    int id, 
    const std::string& mesh_file,
    double scale=1.0,
    double r=1.0, double g=0.0, double b=0.0, double a=1.0,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.mesh_resource = mesh_file;
    return marker;
  }

  /**
   * @brief Create a colored circle marker with radius r
   * 
   * @param id 
   * @param r 
   * @param g 
   * @param b 
   * @return visualization_msgs::Marker 
   */
  inline visualization_msgs::Marker create_circle_marker(
    const std::string& ns, 
    int id, 
    double radius, 
    double r=1.0, double g=0.0, double b=0.0,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
  }

  inline visualization_msgs::Marker create_sphere_marker(
    const std::string& ns, 
    int id, double radius, 
    double r=0.5, double g=0.5, double b=0.5,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  inline visualization_msgs::Marker create_cube_marker(
    const std::string& ns, 
    int id, double length, 
    double r=0.5, double g=0.5, double b=0.5,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = length;
    marker.scale.y = length;
    marker.scale.z = length;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  inline visualization_msgs::Marker create_line_strip_marker(
    const std::string& ns, 
    int id, 
    double width=0.01, 
    double r=0.5, double g=0.5, double b=0.5,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = width;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  inline visualization_msgs::Marker create_line_list_marker(
    const std::string& ns, 
    int id, 
    double width=0.01, 
    double r=0.5, double g=0.5, double b=0.5,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = width;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  inline visualization_msgs::Marker create_sphere_list_marker(
    const std::string& ns, 
    int id, 
    double radius=0.01, 
    double r=0.5, double g=0.5, double b=0.5,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = radius;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  inline visualization_msgs::Marker create_arrow_marker(
    const std::string& ns, 
    int id, 
    double shaft_diameter=0.05,
    double head_diameter=0.07,
    double r=0.5, double g=0.5, double b=0.5,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = shaft_diameter;
    marker.scale.y = head_diameter;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  inline visualization_msgs::Marker create_text_marker(
    const std::string& ns, 
    int id, 
    double scale=0.05,
    double r=0.5, double g=0.5, double b=0.5,
    const std::string& frame="")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = scale;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  inline visualization_msgs::InteractiveMarker create_interactive_marker_pos_orient(
    const std::string& name, 
    const visualization_msgs::Marker& marker,
    const geometry_msgs::Pose& pose,
    double scale=0.15)
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.name = name;
    int_marker.description = "interactive_marker";
    int_marker.scale = scale;

    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    return int_marker;
  }

  inline visualization_msgs::InteractiveMarker create_interactive_marker_pos(
    const std::string& name, 
    const visualization_msgs::Marker& marker,
    const geometry_msgs::Pose& pose,
    double scale=0.15)
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.name = name;
    int_marker.description = "interactive_marker";
    int_marker.scale = scale;

    visualization_msgs::InteractiveMarkerControl control;

    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    return int_marker;
  }

  inline visualization_msgs::InteractiveMarker create_interactive_marker_orient(
    const std::string& name, 
    const visualization_msgs::Marker& marker,
    const geometry_msgs::Pose& pose,
    double scale=0.15)
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.name = name;
    int_marker.description = "interactive_marker";
    int_marker.scale = scale;

    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    return int_marker;
  }


}

#endif