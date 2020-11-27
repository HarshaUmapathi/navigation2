// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: Steve Macenski

#include "dynamic_obstacle_layer/dynamic_obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

PLUGINLIB_EXPORT_CLASS(dynamic_obstacle_layer::DynamicObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace dynamic_obstacle_layer
{

DynamicObstacleLayer::DynamicObstacleLayer()
{
}

DynamicObstacleLayer::~DynamicObstacleLayer()
{
  filter_.reset();
  sub_.reset();
}

void DynamicObstacleLayer::onInitialize()
{
  RCLCPP_INFO(logger_, "Initializing %s of type DynamicObstacleLayer.", name_.c_str());

  // general info
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_ = layered_costmap_->isRolling();
  current_ = true;

  // get topic for tracking information
  std::vector<std::string> topics;
  declaredParameter("tracks_topics", std::vector<std::string>({"dynamic_obstacle_tracks"}));
  get_parameter(name_ + "." + "tracks_topics", topics);

  // depth of tracks subscriber
  int topic_depth;
  declaredParameter("tracks_sub_depth", 10);
  get_parameter(name_ + "." + "tracks_sub_depth", topic_depth);

  // time to project ahead
  declaredParameter("projection_time", 3.0);
  get_parameter(name_ + "." + "projection_time", projection_time_);

  // model to use for projection
  std::string projection_model;
  declaredParameter("projection_model", std::string(""));
  get_parameter(name_ + "." + "projection_model", projection_model);
  if (projection_model == "NONE") {
    projection_model = ProjectionModel::NONE;
  } else if (projection_model == "GAUSSIAN") {
    projection_model = ProjectionModel::GAUSSIAN;
  } else if (projection_model == "SMEAR") {
    projection_model = ProjectionModel::SMEAR;
  } else {
    RCLCPP_WARN(
      logger_, "No valid projection model set (%s), using None by default.",
      projection_model.c_str());
    projection_model = ProjectionModel::NONE;
  }

  // enabled
  declaredParameter("enabled", true);
  get_parameter("enabled", enabled_);

  // combination method
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + "." + "combination_method", combination_method_);

  // transform_tolerance
  node_->get_parameter("transform_tolerance", transform_tolerance);

  // footprint clearing
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);

  // create subscribers with message filters for all requested topics
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = topic_depth;

  for (unsigned int idx = 0; idx != topics.size(); idx++) {
    subs_.push_back(
      std::make_unique<message_filters::Subscriber<nav2_msgs::msg::Tracks>>(
        rclcpp_node_, topics[i], custom_qos_profile));
    subs_.back()->unsubscribe();
    filters_.push_back(std::make_unique<tf2_ros::MessageFilter<nav2_msgs::msg::Tracks>>(
        *subs_.back(), *tf_, global_frame_, topic_depth, rclcpp_node_));
    filters_.back()->registerCallback(
      std::bind(&DynamicObstacleLayer::tracksCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "%s: Subscribed to track topic %s.", name_.c_str(), topics[i].c_str());
  }
}

void DynamicObstacleLayer::tracksCallback(msg)
{
  std::lock_gaurd<std::recursive_mutex> lock(lock_);
  
  for (unsigned int idx = 0; idx != msg.tracks.size(); idx++) {
    // transform into global frame
    if (msg.header.frame_id != global_frame_) {  
      tf2::doTransform(msg.tracks[i].pose, msg.tracks[i].pose);
      tf2::doTransform(msg.tracks[i].velocity, msg.tracks[i].velocity);
      tf2::doTransform(msg.tracks[i].size, msg.tracks[i].size);
    }

    // add tracks to map
    tracks_[msg.tracks[i].id] = msg.tracks[i];
  }
}

void DynamicObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  if (rolling_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  current_ = true;

  // Reset costmap each cycle to recompute based on current tracks
  resetMaps();

  // Each track will be projected into the costmap if reasonably current.
  // This allows us to both replace existing tracks with new information and 
  // decay old tracks no longer around from an arbitrary number of data sources.
  std::lock_gaurd<std::recursive_mutex> lock(lock_);
  Tracks::iterator it = tracks_.begin();
  for (; it != tracks_.end(); ++it) {
    if (clock_->now() - it->second.header.stamp > info_validity_duration_) {
      // delete from map, too old to keep
      RCLCPP_DEBUG(logger_, "Removing track %i, too outdated to process.", it->id);
      tracks_.delete(it);
      continue;
    }

    // project this obstacle
    processObstacle(it->at(idx));
  }
}

void DynamicObstacleLayer::processObstacle(const nav2_msgs::msg::Track & obj)
{
  // Use project model selected to insert dynamic obstacles into costmap
  case projection_model_:
    case ProjectionModel::NONE:
      insertObstacle(obj);
      break;
    case ProjectionModel::GAUSSIAN:
      gaussianProcessObstacle(obj);
      break;
    case ProjectionModel::SMEAR:
      smearObstacle(obj);
      break;
    case ProjectionModel::TODO:
  // TODO create own new one using the lessons learned from these above!
      break;
    default:
      break;
}

void DynamicObstacleLayer::insertObstacle(const nav2_msgs::msg::Track & obj)
{
  // directly insert obstacle's size at pose

  // get box made up of size at pose in center
  double max_x = pose + size_x / 2;  // TODO compute poses
  double max_y = pose + size_y / 2;
  double min_x = pose - size_x / 2;
  double min_y = pose - size_y / 2;
  min_x = std::min(min_x, max_x);
  max_x = std::max(min_x, max_x);
  min_y = std::min(min_y, max_y);
  max_y = std::max(min_y, max_y);

  // mark lethal inside box in costmap
  const double resolution = layered_costmap_->getResolution();
  unsigned int cost_i, cost_j;
  for (unsigned int i = min_x; i <= max_x; i += resolution) {
    for (unsigned int j = min_y; j <= max_y; j += resolution) {
      costmap_->worldToMap(i, j, cost_i, cost_j);
      costmap_->setCost(cost_i, cost_j, LETHAL_OBSTACLE);
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
}

void DynamicObstacleLayer::smearObstacle(const nav2_msgs::msg::Track & obj)
{
  // insert obstacle and smear' forward in time at intervals
  nav2_msgs::msg::Track obj_projection = obj;
  double sim_time = 0.5;  // seconds TODO param
  const unsigned int increments = static_cast<unsigned int>(projection_time_ / sim_time);

  // move object's pose by sim_tim in velocity direction and mark
  for (unsigned int idx = 0; idx != increments; idx++) {
    obj_projection.pose = obj_projection.pose in velocity direction sim_time * i  // TODO kinematics
    insertObstacle(obj_projection);
  }
}

// https://ieeexplore.ieee.org/document/7978468

void DynamicObstacleLayer::gaussianProcessObstacle(const nav2_msgs::msg::Track & obj)
{
  // create a guassian distribution at obstacle's pose with principle
  // component of the velocity direction, scaled by time and size.
  // projection models for gaussian:
  // https://alyssapierson.files.wordpress.com/2018/05/pierson2018.pdf
  // http://ras.papercept.net/images/temp/IROS/files/1999.pdf
  // https://ieeexplore.ieee.org/document/8967744
  // project and mark where going to be based on model selected in layer's costmap
  // touch(px, py, min_x, min_y, max_x, max_y); to update bounds

}

void DynamicObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  
  // If footprint clearing is enabled, clear the footprint polygon on costmap
  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  // Combine this costmap's information with the layered costmap's master grid
  switch (combination_method_) {
    case 0:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:
      break;
  }
}

void DynamicObstacleLayer::activate()
{
  sub_->subscribe();
}

void DynamicObstacleLayer::deactivate()
{
  sub_->unsubscribe();
  tracks_.clear();
}

void DynamicObstacleLayer::reset()
{
  deactivate();
  activate();
  current_ = false;
}

void DynamicObstacleLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void DynamicObstacleLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // update the origin with the appropriate world coordinates
  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

}  // namespace dynamic_obstacle_layer
