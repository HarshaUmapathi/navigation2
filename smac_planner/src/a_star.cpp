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

#include <omp.h>

#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <chrono>
#include <thread>
#include <utility>

#include "smac_planner/a_star.hpp"
using namespace std::chrono;  // NOLINT

namespace smac_planner
{

template<typename NodeT>
AStarAlgorithm<NodeT>::AStarAlgorithm(
  const MotionModel & motion_model,
  const SearchInfo & search_info)
: _traverse_unknown(true),
  _max_iterations(0),
  _x_size(0),
  _y_size(0),
  _search_info(search_info),
  _goal_coordinates(Coordinates()),
  _start(nullptr),
  _goal(nullptr),
  _motion_model(motion_model),
  _collision_checker(nullptr)
{
  _graph.reserve(100000);
}

template<typename NodeT>
AStarAlgorithm<NodeT>::~AStarAlgorithm()
{
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations)
{
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
}

template<>
void AStarAlgorithm<Node2D>::createGraph(
  const unsigned int & x_size,
  const unsigned int & y_size,
  const unsigned int & dim_3_size,
  nav2_costmap_2d::Costmap2D * & costmap)
{
  if (dim_3_size != 1) {
    throw std::runtime_error("Node type Node2D cannot be given non-1 dim 3 quantization.");
  }
  _costmap = costmap;
  _dim3_size = dim_3_size;  // 2D search MUST be 2D, not 3D or SE2.
  clearGraph();

  if (getSizeX() != x_size || getSizeY() != y_size) {
    _x_size = x_size;
    _y_size = y_size;
    Node2D::initNeighborhood(_x_size, _motion_model);
  }
}

template<>
void AStarAlgorithm<NodeSE2>::createGraph(
  const unsigned int & x_size,
  const unsigned int & y_size,
  const unsigned int & dim_3_size,
  nav2_costmap_2d::Costmap2D * & costmap)
{
  _costmap = costmap;
  _collision_checker = GridCollisionChecker(costmap);
  _collision_checker.setFootprint(_footprint, _is_radius_footprint);

  _dim3_size = dim_3_size;
  unsigned int index;
  clearGraph();

  if (getSizeX() != x_size || getSizeY() != y_size) {
    _x_size = x_size;
    _y_size = y_size;
    NodeSE2::initMotionModel(_motion_model, _x_size, _y_size, _dim3_size, _search_info);
  }
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setFootprint(nav2_costmap_2d::Footprint footprint, bool use_radius)
{
  _footprint = footprint;
  _is_radius_footprint = use_radius;
}

template<>
typename AStarAlgorithm<Node2D>::NodePtr AStarAlgorithm<Node2D>::addToGraph(
  const unsigned int & index)
{
  return &(_graph.emplace(index, Node2D(_costmap->getCharMap()[index], index)).first->second);
}

template<>
typename AStarAlgorithm<NodeSE2>::NodePtr AStarAlgorithm<NodeSE2>::addToGraph(
  const unsigned int & index)
{
  return &(_graph.emplace(index, NodeSE2(index)).first->second);
}

template<>
void AStarAlgorithm<Node2D>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero starting dim 3.");
  }
  _start = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
}

template<>
void AStarAlgorithm<NodeSE2>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  _start = addToGraph(NodeSE2::getIndex(mx, my, dim_3, getSizeX(), getSizeDim3()));
  _start->setPose(
    Coordinates(
      static_cast<float>(mx),
      static_cast<float>(my),
      static_cast<float>(dim_3)));
}

template<>
void AStarAlgorithm<Node2D>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero goal dim 3.");
  }

  _goal = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
  _goal_coordinates = Node2D::Coordinates(mx, my);
}

template<>
void AStarAlgorithm<NodeSE2>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  _goal = addToGraph(NodeSE2::getIndex(mx, my, dim_3, getSizeX(), getSizeDim3()));
  _goal_coordinates = NodeSE2::Coordinates(
    static_cast<float>(mx),
    static_cast<float>(my),
    static_cast<float>(dim_3));
  _goal->setPose(_goal_coordinates);

  NodeSE2::computeWavefrontHeuristic(
    _costmap,
    static_cast<unsigned int>(getStart()->pose.x),
    static_cast<unsigned int>(getStart()->pose.y),
    mx, my);
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::areInputsValid()
{
  // Check if graph was filled in
  if (_graph.empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!_start || !_goal) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  if (getToleranceHeuristic() < 0.001 &&
    !_goal->isNodeValid(_traverse_unknown, _collision_checker))
  {
    throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
  }

  // Check if starting point is valid
  if (!_start->isNodeValid(_traverse_unknown, _collision_checker)) {
    throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
  }

  return true;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::createPath(
  CoordinateVector & path, int & iterations,
  const float & tolerance)
{
  _tolerance = tolerance * NodeT::neutral_cost;
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
  clearQueue();

  if (!areInputsValid()) {
    return false;
  }

  // 0) Add starting point to the open set
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0);

  // Optimization: preallocate all variables
  NodePtr current_node = nullptr;
  NodePtr neighbor = nullptr;
  float g_cost = 0.0;
  NodeVector neighbors;
  int approach_iterations = 0;
  NeighborIterator neighbor_iterator;

  // Given an index, return a node ptr reference if its collision-free and valid
  const unsigned int max_index = getSizeX() * getSizeY() * getSizeDim3();
  std::function<bool(const unsigned int &, NodeT * &)> neighborGetter =
    [&, this](const unsigned int & index, NodePtr & neighbor_rtn) -> bool
    {
      if (index < 0 || index >= max_index) {
        return false;
      }

      neighbor_rtn = addToGraph(index);
      return true;
    };

  while (iterations < getMaxIterations() && !_queue.empty()) {
    // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    current_node = getNextNode();

    // We allow for nodes to be queued multiple times in case
    // shorter paths result in it, but we can visit only once
    if (current_node->wasVisited()) {
      continue;
    }

    iterations++;

    // 2) Mark Nbest as visited
    current_node->visited();

    // 3) Check if we're at the goal, backtrace if required
    if (isGoal(current_node)) {
      return backtracePath(current_node, path);
    } else if (_best_heuristic_node.first < getToleranceHeuristic()) {
      // Optimization: Let us find when in tolerance and refine within reason
      approach_iterations++;
      if (approach_iterations > getOnApproachMaxIterations() ||
        iterations + 1 == getMaxIterations())
      {
        NodePtr node = &_graph.at(_best_heuristic_node.second);
        return backtracePath(node, path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    NodeT::getNeighbors(
      current_node, neighborGetter, _collision_checker, _traverse_unknown, neighbors);

    for (neighbor_iterator = neighbors.begin();
      neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      neighbor = *neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      g_cost = getAccumulatedCost(current_node) + getTraversalCost(current_node, neighbor);

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < getAccumulatedCost(neighbor)) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;

        // 4.3) If not in queue or visited, add it, `getNeighbors()` handles
        neighbor->queued();
        addNode(g_cost + getHeuristicCost(neighbor), neighbor);
      }
    }
  }

  return false;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::isGoal(NodePtr & node)
{
  return node == getGoal();
}

template<>
bool AStarAlgorithm<Node2D>::backtracePath(NodePtr & node, CoordinateVector & path)
{
  if (!node->parent) {
    return false;
  }

  NodePtr current_node = node;

  while (current_node->parent) {
    path.push_back(
      Node2D::getCoords(
        current_node->getIndex(), getSizeX(), getSizeDim3()));
    current_node = current_node->parent;
  }

  return path.size() > 1;
}

template<>
bool AStarAlgorithm<NodeSE2>::backtracePath(NodePtr & node, CoordinateVector & path)
{
  if (!node->parent) {
    return false;
  }

  NodePtr current_node = node;

  while (current_node->parent) {
    path.push_back(current_node->pose);
    current_node = current_node->parent;
  }

  return path.size() > 1;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getStart()
{
  return _start;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getGoal()
{
  return _goal;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::getNextNode()
{
  NodeBasic<NodeT> node = _queue.top().second;
  _queue.pop();
  return node.graph_node_ptr;
}

template<>
typename AStarAlgorithm<NodeSE2>::NodePtr AStarAlgorithm<NodeSE2>::getNextNode()
{
  NodeBasic<NodeSE2> node = _queue.top().second;
  _queue.pop();

  if (!node.graph_node_ptr->wasVisited()) {
    node.graph_node_ptr->pose = node.pose;
  }

  return node.graph_node_ptr;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::addNode(const float cost, NodePtr & node)
{
  NodeBasic<NodeT> queued_node(node->getIndex());
  queued_node.graph_node_ptr = node;
  _queue.emplace(cost, queued_node);
}

template<>
void AStarAlgorithm<NodeSE2>::addNode(const float cost, NodePtr & node)
{
  NodeBasic<NodeSE2> queued_node(node->getIndex());
  queued_node.pose = node->pose;
  queued_node.graph_node_ptr = node;
  _queue.emplace(cost, queued_node);
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getTraversalCost(
  NodePtr & current_node,
  NodePtr & new_node)
{
  return current_node->getTraversalCost(new_node);
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getAccumulatedCost(NodePtr & node)
{
  return node->getAccumulatedCost();
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getHeuristicCost(const NodePtr & node)
{
  const Coordinates node_coords =
    NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  float heuristic = NodeT::getHeuristicCost(
    node_coords, _goal_coordinates);

  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};
  }

  return heuristic;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearQueue()
{
  NodeQueue q;
  std::swap(_queue, q);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearGraph()
{
  Graph g;
  g.reserve(100000);
  std::swap(_graph, g);
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getMaxIterations()
{
  return _max_iterations;
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getOnApproachMaxIterations()
{
  return _max_on_approach_iterations;
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getToleranceHeuristic()
{
  return _tolerance;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeX()
{
  return _x_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeY()
{
  return _y_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeDim3()
{
  return _dim3_size;
}

// Instantiate algorithm for the supported template types
template class AStarAlgorithm<Node2D>;
template class AStarAlgorithm<NodeSE2>;

}  // namespace smac_planner
