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

#include <control_core/ros/trajectory_publisher.h>

namespace cc
{

TrajectoryPublisher::TrajectoryPublisher() :
  state_(CONSTRUCTED),
  max_size_(0),
  d_theshold_(0.01)
{
}

TrajectoryPublisher::~TrajectoryPublisher()
{
}

bool TrajectoryPublisher::advertise(
  ros::NodeHandle& nh, 
  const std::string& name,
  const std::string& frame, 
  const ros::Duration& hist_dur,
  const ros::Duration& period)
{
  if(state_ != INITIALIZED)
  {
    state_ = INITIALIZED;

    // setup publisher
    name_ = name;
    frame_ = frame;
    hist_dur_ = hist_dur;
    period_ = period;
    path_pub_ = nh.advertise<nav_msgs::Path>(name_, 1);
    pose_pub_ = nh.advertise<geometry_msgs::PointStamped>(name_ + "_cur", 1);

    // setup message
    path_.header.frame_id = frame_;
    return true;
  }
  return false;
}

bool TrajectoryPublisher::advertise(
  ros::NodeHandle& nh, 
  const std::string& name,
  const std::string& frame, 
  size_t max_size,
  const ros::Duration& period)
{
  if(state_ != INITIALIZED)
  {
    state_ = INITIALIZED;

    // setup publisher
    name_ = name;
    frame_ = frame;
    max_size_ = max_size;
    period_ = period;
    path_pub_ = nh.advertise<nav_msgs::Path>(name_, 1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(name_ + "_cur", 1);

    // setup message
    path_.header.frame_id = frame_;
    pose_.header.frame_id = frame_;
    return true;
  }
  return false;
}


void TrajectoryPublisher::update(
  const cc::LinearPosition& x,
  const ros::Time& time)
{
  cc::CartesianPosition X;
  X.linear() = x;
  X.angular().setIdentity();
  update(X, time);
}

void TrajectoryPublisher::update(
  const cc::CartesianPosition& X, 
  const ros::Time& time)
{
  if(!data_.empty())
  {
    if((time - data_.back().header.stamp) < period_)
    {
      // data point to close
      return;
    }
  }

  // add pose
  pose_.pose = X;
  pose_.header.stamp = time;
  data_.push_back(pose_);

  // remove old elements
  if(max_size_ > 0)
  {
    // based on max_size_ elements
    while(data_.size() > max_size_)
    {
      data_.pop_front();
    }
  }
  else
  {
    // based on the elapsed time in hist_dur_
    while(time - data_.front().header.stamp > hist_dur_)
    {
      data_.pop_front();
    }
  }
}   

void TrajectoryPublisher::publish(const ros::Time& time)
{
  path_.header.stamp = time;
  path_.poses = {data_.begin(), data_.end()};
  cc::publish_if_subscribed(path_pub_, path_);
  cc::publish_if_subscribed(pose_pub_, pose_);
}

void TrajectoryPublisher::publishIfSubscribed(const ros::Time& time)
{
  if(path_pub_.getNumSubscribers() > 0 || pose_pub_.getNumSubscribers() > 0)
  {
    publish(time);
  }
}

bool TrajectoryPublisher::checkDistanceTheshold(
  const cc::LinearPosition& x1,
  const geometry_msgs::Point& x2)
{
  return (x1 - cc::LinearPosition(x2.x, x2.y, x2.z)).norm() > d_theshold_;
}

}