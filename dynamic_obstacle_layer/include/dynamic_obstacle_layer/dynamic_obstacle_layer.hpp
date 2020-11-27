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

#ifndef DYNAMIC_OBSTACLE_LAYER__DYNAMIC_OBSTACLE_LAYER_HPP_
#define DYNAMIC_OBSTACLE_LAYER__DYNAMIC_OBSTACLE_LAYER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace dynamic_obstacle_layer
{

enum class ProjectionModel
{
  UNKNOWN = 0,
  NONE = 1,
  GAUSSIAN = 2,
  SMEAR = 3,
  TODO = 4
};

/**
 * @class dynamic_obstacle_layer::DynamicObstacleLayer
 * @brief A costmap layer taking in dynamic tracked information and
 * including it in the costmap projected through time as a risk map
 */
class DynamicObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  typedef std::unordered_map<std::string, nav2_msgs::msg::Tracks> Tracks;
  /**
   * @brief A constructor for dynamic_obstacle_layer::DynamicObstacleLayer
   */
  DynamicObstacleLayer();
  /**
   * @brief A destructor for dynamic_obstacle_layer::DynamicObstacleLayer
   */
  virtual ~DynamicObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

protected:
  void updateFootprint(double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);

  std::vector<std::unique_ptr<message_filters::Subscriber<nav2_msgs::msg::Tracks>>> subs_;
  std::vector<std::unique_ptr<tf2_ros::MessageFilter<nav2_msgs::msg::Tracks>>> filters_;
  Tracks tracks_;

  bool rolling_, footprint_clearing_enabled_;
  double projection_time_;
  ProjectionModel projection_model_;
  std::recursive_mutex lock_;
  std::string global_frame_;
};

}  // namespace dynamic_obstacle_layer

#endif  // DYNAMIC_OBSTACLE_LAYER__DYNAMIC_OBSTACLE_LAYER_HPP_
