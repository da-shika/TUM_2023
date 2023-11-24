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

#ifndef CONTROL_CORE_ROS_TRAJECTORY_PUBLISHER_H
#define CONTROL_CORE_ROS_TRAJECTORY_PUBLISHER_H

#include <control_core/types.h>
#include <control_core/ros/ros.h>
#include <deque>
#include <nav_msgs/Path.h>

namespace cc
{

/*!
 * \brief The TrajectoryPublisher class
 *
 * This class publishes trajectories in rviz.
 */
class TrajectoryPublisher
{
public:
  enum State {CONSTRUCTED, INITIALIZED, RUNNING};

public:
  State state_;

  std::string name_;
  std::string frame_;

  ros::Duration hist_dur_;
  ros::Duration period_;
  size_t max_size_;
  cc::Scalar d_theshold_;

  ros::Publisher path_pub_;
  ros::Publisher pose_pub_;

  nav_msgs::Path path_;
  geometry_msgs::PoseStamped pose_;
  
  std::deque<geometry_msgs::PoseStamped> data_;

public:
  /*!
  * \brief TrajectoryPublisher Default constructor.
  */
  TrajectoryPublisher();

  // destructor
  virtual ~TrajectoryPublisher();

  /*
   * \brief advertise the trajectory.
   * 
   * Limit the number of elements in the trajectory to a history_duration of
   * seconds.
   */
  bool advertise(
    ros::NodeHandle& nh, 
    const std::string& name,
    const std::string& frame, 
    const ros::Duration& hist_dur,
    const ros::Duration& period=ros::Duration(1./30.));

  /**
   * \brief advertise the trajectory
   * 
   * Limit the number of elements in the trajectory to max_size
   * elements.
   */
  bool advertise(
    ros::NodeHandle& nh, 
    const std::string& name,
    const std::string& frame, 
    size_t max_size,
    const ros::Duration& period=ros::Duration(1./30.));

  /** 
   * \brief adds a new cartesian pose to the trajectory
   *
   * \param current jointstate
   * \param current time
   */
  void update(
    const cc::CartesianPosition& X,
    const ros::Time& time);

  /** 
   * \brief adds a new cartesian pose to the trajectory
   *
   * \param current jointstate
   * \param current time
   */
  void update(
    const cc::LinearPosition& x,
    const ros::Time& time);
  
  /** 
  * \brief publish the information
  *
  * \param current time
  */
  void publish(const ros::Time& time);

  /** 
  * \brief publish the information
  *
  * \param current time
  */
  void publishIfSubscribed(const ros::Time& time);

private:
  bool checkDistanceTheshold(
    const cc::LinearPosition& x1,
    const geometry_msgs::Point& x2);

};

}

#endif