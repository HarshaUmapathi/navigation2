// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#include <math.h>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include "smac_planner/node_se2.hpp"

namespace smac_planner
{

// defining static member for all instance to share
MotionTable NodeSE2::_motion_model;
double NodeSE2::neutral_cost = sqrt(2);

// Each of these tables are the projected motion models through
// time and space applied to the search on the current node in
// continuous map-coordinates (e.g. not meters but partial map cells)
// Currently, these are set to project *at minimum* into a neighboring
// cell. Though this could be later modified to project a certain
// amount of time or particular distance forward.

// http://planning.cs.uiuc.edu/node821.html
// Model for ackermann style vehicle with minimum radius restriction
void MotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;

  // angle must meet 3 requirements:
  // 1) be increment of quantized bin size
  // 2) chord length must be greater than sqrt(2) to leave current cell
  // 3) maximum curvature must be respected, represented by minimum turning angle
  // Thusly:
  // On circle of radius minimum turning angle, we need select motion primatives
  // with chord length > sqrt(2) and be an increment of our bin size
  //
  // chord >= sqrt(2) >= 2 * R * sin (angle / 2); where angle / N = quantized bin size
  // Thusly: angle <= 2.0 * asin(sqrt(2) / (2 * R))
  float angle = 2.0 * asin(sqrt(2.0) / (2 * search_info.minimum_turning_radius));
  // Now make sure angle is an increment of the quantized bin size
  // And since its based on the minimum chord, we need to make sure its always larger
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
    angle = bin_size;
  }

  // Search dimensions not promised to be clean multiples of quantization
  increments = angle / bin_size;

  // find deflections
  // If we make a right triangle out of the chord in circle of radius
  // min turning angle, we can see that delta X = R * sin (angle)
  float delta_x = search_info.minimum_turning_radius * sin(angle);
  // Using that same right triangle, we can see that the complement
  // to delta Y is R * cos (angle). If we subtract R, we get the actual value
  float delta_y = search_info.minimum_turning_radius -
    (search_info.minimum_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(3);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
  projections.emplace_back(delta_x, delta_y, increments);  // Left
  projections.emplace_back(delta_x, -delta_y, -increments);  // Right

  // Create the correct OMPL state space
  state_space = std::make_unique<ompl::base::DubinsStateSpace>(search_info.minimum_turning_radius);
}

// http://planning.cs.uiuc.edu/node822.html
// Same as Dubin model but now reverse is valid
// See notes in Dubin for explanation
void MotionTable::initReedsShepp(
  unsigned int & size_x_in,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;

  float angle = 2.0 * asin(sqrt(2.0) / (2 * search_info.minimum_turning_radius));
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
    angle = bin_size;
  }

  increments = angle / bin_size;
  float delta_x = search_info.minimum_turning_radius * sin(angle);
  float delta_y = search_info.minimum_turning_radius -
    (search_info.minimum_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(6);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
  projections.emplace_back(delta_x, delta_y, increments);  // Forward + Left
  projections.emplace_back(delta_x, -delta_y, -increments);  // Forward + Right
  projections.emplace_back(-hypotf(delta_x, delta_y), 0.0, 0.0);  // Backward
  projections.emplace_back(-delta_x, delta_y, -increments);  // Backward + Left
  projections.emplace_back(-delta_x, -delta_y, increments);  // Backward + Right

  // Create the correct OMPL state space
  state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(
    search_info.minimum_turning_radius);
}

MotionPoses MotionTable::getProjections(const NodeSE2 * node)
{
  MotionPoses projection_list;
  for (unsigned int i = 0; i != projections.size(); i++) {
    projection_list.push_back(getProjection(node, i));
  }

  return projection_list;
}

MotionPose MotionTable::getProjection(const NodeSE2 * node, const unsigned int & motion_index)
{
  const MotionPose & motion_model = projections[motion_index];

  // transform delta X, Y, and Theta into local coordinates
  const float & node_heading = node->pose.theta;
  const float cos_theta = cos(node_heading * bin_size);  // needs actual angle [0, 2PI]
  const float sin_theta = sin(node_heading * bin_size);
  const float delta_x = motion_model._x * cos_theta - motion_model._y * sin_theta;
  const float delta_y = motion_model._x * sin_theta + motion_model._y * cos_theta;
  float new_heading = node_heading + motion_model._theta;

  // normalize theta
  while (new_heading >= num_angle_quantization_float) {
    new_heading -= num_angle_quantization_float;
  }
  while (new_heading < 0.0) {
    new_heading += num_angle_quantization_float;
  }

  return MotionPose(delta_x + node->pose.x, delta_y + node->pose.y, new_heading);
}

NodeSE2::NodeSE2(GridCollisionChecker * collision_checker, const unsigned int index)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _is_queued(false),
  _collision_checker(collision_checker),
  _motion_primitive_index(std::numeric_limits<unsigned int>::max())
{
}

NodeSE2::~NodeSE2()
{
  parent = nullptr;
}

void NodeSE2::reset(GridCollisionChecker * collision_checker)
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  _is_queued = false;
  _collision_checker = collision_checker;
  _motion_primitive_index = std::numeric_limits<unsigned int>::max();
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
}

bool NodeSE2::isNodeValid(const bool & traverse_unknown)
{
  const float angle_in_rad = this->pose.theta * _motion_model.bin_size;
  if (_collision_checker->inCollision(
      this->pose.x, this->pose.y, angle_in_rad, traverse_unknown))
  {
    return false;
  }

  _cell_cost = _collision_checker->getCost();
  return true;
}

float NodeSE2::getTraversalCost(const NodePtr & child)
{
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
            "Node attempted to get traversal "
            "cost without a known SE2 collision cost!");
  }

  // this is the first node
  if (getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max()) {
    return NodeSE2::neutral_cost;
  }

  float travel_cost = 0.0;
  float travel_cost_raw = NodeSE2::neutral_cost + _motion_model.cost_penalty * normalized_cost;

  if (getMotionPrimitiveIndex() == 0 || getMotionPrimitiveIndex() == 3) {
    // straight motion, no additional costs to be applied
    travel_cost = travel_cost_raw;
  } else {
    if (getMotionPrimitiveIndex() == child->getMotionPrimitiveIndex()) {
      // Turning motion but keeps in same direction: encourages to commit to turning if starting it
      travel_cost = travel_cost_raw * _motion_model.non_straight_penalty;
    } else {
      // Turning motion and changing direction: penalizes wiggling
      travel_cost = travel_cost_raw * _motion_model.change_penalty;
      travel_cost += travel_cost_raw * _motion_model.non_straight_penalty;
    }
  }

  if (getMotionPrimitiveIndex() > 2) {
    // reverse direction
    travel_cost *= _motion_model.reverse_penalty;
  }

  return travel_cost;
}

float NodeSE2::getHeuristicCost(
  const Coordinates & node_coords,
  const Coordinates & goal_coords)
{
  // Dubin or Reeds-Shepp shortest distances
  ompl::base::ScopedState<> from(_motion_model.state_space), to(_motion_model.state_space);
  from[0] = node_coords.x;
  from[1] = node_coords.y;
  from[2] = node_coords.theta * _motion_model.bin_size;
  to[0] = goal_coords.x;
  to[1] = goal_coords.y;
  to[2] = goal_coords.theta * _motion_model.bin_size;

  const float model_heuristic = _motion_model.state_space->distance(from(), to());
  const float euclidean_heuristic = hypotf(goal_coords.x - node_coords.x, goal_coords.y - node_coords.y);
  return NodeSE2::neutral_cost * std::max(euclidean_heuristic, model_heuristic);
}

void NodeSE2::initMotionModel(
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & num_angle_quantization,
  SearchInfo & search_info)
{
  // find the motion model selected
  switch (motion_model) {
    case MotionModel::DUBIN:
      _motion_model.initDubin(size_x, num_angle_quantization, search_info);
      break;
    case MotionModel::REEDS_SHEPP:
      _motion_model.initReedsShepp(size_x, num_angle_quantization, search_info);
      break;
    default:
      throw std::runtime_error(
              "Invalid motion model for SE2 node. Please select between"
              " Dubin (Ackermann forward only),"
              " Reeds-Shepp (Ackermann forward and back),"
              " or Balkcom-Mason (Differential drive and omnidirectional) models.");
  }
}

void NodeSE2::getNeighbors(
  const NodePtr & node,
  std::function<bool(const unsigned int &, smac_planner::NodeSE2 * &)> & NeighborGetter,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  unsigned int index = 0;
  NodePtr neighbor = nullptr;
  const MotionPoses motion_projections = _motion_model.getProjections(node);
  Coordinates initial_node_coords;

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeSE2::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta),
      _motion_model.size_x, _motion_model.num_angle_quantization);

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited() && !neighbor->isQueued()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown)) {
        neighbor->setMotionPrimitiveIndex(i);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

}  // namespace smac_planner
