#include <ompl/base/ScopedState.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>

#include "path_searching/hybrid_a.h"

using namespace std::chrono;

AStarAlgorithm::AStarAlgorithm()
    : _traverse_unknown(true), _max_iterations(0), _x_size(0), _y_size(0), _goal_coordinates(Coordinates()), _start(nullptr), _goal(nullptr) {
  _graph.reserve(100000);
}

AStarAlgorithm::~AStarAlgorithm() {}

void AStarAlgorithm::initialize(const bool allow_unknown, int max_iterations, const float lookup_table_size, const unsigned int dim_3_size,
                                nav_msgs::OccupancyGridPtr grid_map, float min_turning_radius) {
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _dim3_size = dim_3_size;
  NodeHybrid::precomputeDistanceHeuristic(1.0, _dim3_size, min_turning_radius);
  _grid_map = grid_map;

  clearGraph();
  _x_size = grid_map->info.width;
  _y_size = grid_map->info.height;
  NodeHybrid::initMotionModel(_x_size, _y_size, _dim3_size, min_turning_radius, grid_map->info.resolution);
}

AStarAlgorithm::NodePtr AStarAlgorithm::addToGraph(const unsigned int& index) {
  // Emplace只会在元素不存在的时候创建新的元素
  // 如果存在，就会返回该存在的对象，而不会再创建
  // 返回值是std::pair<iterator, bool>
  return &(_graph.emplace(index, NodeHybrid(index)).first->second);
}

void AStarAlgorithm::setStart(const unsigned int& mx, const unsigned int& my, const unsigned int& dim_3, const float wx_in_map,
                              const float wy_in_map) {
  _start = addToGraph(NodeHybrid::getIndex(mx, my, dim_3));
  _start->setPose(Coordinates(static_cast<float>(wx_in_map), static_cast<float>(wy_in_map), static_cast<float>(dim_3)));
}

void AStarAlgorithm::setGoal(const unsigned int& mx, const unsigned int& my, const unsigned int& dim_3, const float wx_in_map,
                             const float wy_in_map) {
  _goal = addToGraph(NodeHybrid::getIndex(mx, my, dim_3));

  typename NodeHybrid::Coordinates goal_coords(static_cast<float>(wx_in_map), static_cast<float>(wy_in_map), static_cast<float>(dim_3));

  if (!_cache_obstacle_heuristic || goal_coords != _goal_coordinates) {
    NodeHybrid::resetObstacleHeuristic(_grid_map, mx, my);
  }

  _goal_coordinates = goal_coords;
  _goal->setPose(_goal_coordinates);
}

bool AStarAlgorithm::areInputsValid() {
  // Check if graph was filled in
  if (_graph.empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // 检查是否为空指针
  if (!_start || !_goal) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  if (getToleranceHeuristic() < 0.001 && !_goal->isNodeValid(_grid_map)) {
    throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
  }

  // Check if starting point is valid
  if (!_start->isNodeValid(_grid_map)) {
    throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
  }

  return true;
}

bool AStarAlgorithm::createPath(std::vector<Coordinates>& path, int& iterations, const float& tolerance) {
  _tolerance = tolerance;
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
  clearQueue();

  if (!areInputsValid()) {
    return false;
  }

  // 0) 把起始点添加到openset中
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0);

  // Optimization: preallocate all variables
  NodePtr current_node = nullptr;
  NodePtr neighbor = nullptr;
  NodePtr expansion_result = nullptr;
  float g_cost = 0.0;
  std::vector<NodePtr> neighbors;
  int approach_iterations = 0;
  NeighborIterator neighbor_iterator;
  int analytic_iterations = 0;
  int closest_distance = std::numeric_limits<int>::max();
  const unsigned int max_index = getSizeX() * getSizeY() * getSizeDim3();

  // Given an index, return a node ptr reference if its collision-free and valid
  NodeGetter neighborGetter = [&, this](const unsigned int& index, NodePtr& neighbor_rtn) -> bool {
    if (index < 0 || index >= max_index) {
      return false;
    }

    neighbor_rtn = addToGraph(index);
    return true;
  };

  // 算法主要循环
  while (iterations < getMaxIterations() && !_queue.empty()) {
    // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    current_node = getNextNode();

    // We allow for nodes to be queued multiple times in case
    // shorter paths result in it, but we can visit only once
    // 允许节点被多次排序来获得更近的path，但只允许访问一次
    if (current_node->wasVisited()) {
      continue;
    }

    iterations++;

    // 2) Mark Nbest as visited
    current_node->visited();

    // 2.1) Use an analytic expansion (if available) to generate a path
    expansion_result = nullptr;
    expansion_result = tryAnalyticExpansion(current_node, neighborGetter, analytic_iterations, closest_distance);
    if (expansion_result != nullptr) {
      current_node = expansion_result;
    }

    // 3) Check if we're at the goal, backtrace if required
    if (isGoal(current_node)) {
      return backtracePath(current_node, path);
    } else if (_best_heuristic_node.first < getToleranceHeuristic()) {
      // Optimization: Let us find when in tolerance and refine within reason
      approach_iterations++;
      if (approach_iterations >= getOnApproachMaxIterations()) {
        return backtracePath(&_graph.at(_best_heuristic_node.second), path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    current_node->getNeighbors(neighborGetter, _traverse_unknown, neighbors, _grid_map);

    for (neighbor_iterator = neighbors.begin(); neighbor_iterator != neighbors.end(); ++neighbor_iterator) {
      neighbor = *neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      // 每个节点的gcost = 上一个节点的gcost + 上一个节点到当前节点的cost
      g_cost = current_node->getAccumulatedCost() + current_node->getTraversalCost(neighbor);

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;
      }
      // 4.3) Add to queue with heuristic cost
      addNode(g_cost + getHeuristicCost(neighbor), neighbor);
    }
  }

  return false;
}

bool AStarAlgorithm::isGoal(NodePtr& node) { return node == getGoal(); }

bool AStarAlgorithm::backtracePath(NodePtr node, std::vector<Coordinates>& path) {
  if (!node->parent) {
    return false;
  }

  NodePtr current_node = node;

  while (current_node->parent) {
    path.push_back(current_node->pose);
    current_node = current_node->parent;
  }

  return path.size() > 0;
}

AStarAlgorithm::NodePtr& AStarAlgorithm::getStart() { return _start; }

AStarAlgorithm::NodePtr& AStarAlgorithm::getGoal() { return _goal; }

AStarAlgorithm::NodePtr AStarAlgorithm::getNextNode() {
  NodeHybrid node = _queue.top().second;
  _queue.pop();

  // We only want to override the node's pose if it has not yet been visited
  // to prevent the case that a node has been queued multiple times and
  // a new branch is overriding one of lower cost already visited.
  // 只在节点未被访问时重写节点的pose
  if (!node.graph_node_ptr->wasVisited()) {
    node.graph_node_ptr->pose = node.pose;
  }

  return node.graph_node_ptr;
}

/**
 * @brief 将node添加进openset中
 *
 * @param cost
 * @param node
 */
void AStarAlgorithm::addNode(const float& cost, NodePtr& node) {
  NodeHybrid queued_node(node->getIndex());
  queued_node.pose = node->pose;
  queued_node.graph_node_ptr = node;
  _queue.emplace(cost, queued_node);
}

float AStarAlgorithm::getHeuristicCost(NodePtr& node) {
  const Coordinates node_coords = NodeHybrid::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  float heuristic = NodeHybrid::getHeuristicCost(node_coords, _goal_coordinates, _grid_map);

  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};
  }

  return heuristic;
}

void AStarAlgorithm::clearQueue() {
  NodeQueue q;
  std::swap(_queue, q);
}

void AStarAlgorithm::clearGraph() {
  std::unordered_map<unsigned int, NodeHybrid> g;
  std::swap(_graph, g);
  _graph.reserve(100000);
}

int& AStarAlgorithm::getMaxIterations() { return _max_iterations; }

int& AStarAlgorithm::getOnApproachMaxIterations() { return _max_on_approach_iterations; }

float& AStarAlgorithm::getToleranceHeuristic() { return _tolerance; }

unsigned int& AStarAlgorithm::getSizeX() { return _x_size; }

unsigned int& AStarAlgorithm::getSizeY() { return _y_size; }

unsigned int& AStarAlgorithm::getSizeDim3() { return _dim3_size; }

AStarAlgorithm::NodePtr AStarAlgorithm::tryAnalyticExpansion(const NodePtr& current_node, const NodeGetter& getter, int& analytic_iterations,
                                                             int& closest_distance) {
  const Coordinates node_coords = NodeHybrid::getCoords(current_node->getIndex(), getSizeX(), getSizeDim3());
  // 计算目前为止，到目标点的最小的distance（obs_h和dis_h的最大值）
  closest_distance = std::min(closest_distance, static_cast<int>(NodeHybrid::getHeuristicCost(node_coords, _goal_coordinates, _grid_map)));

  // We want to expand at a rate of d/expansion_ratio,
  // but check to see if we are so close that we would be expanding every iteration
  // If so, limit it to the expansion ratio (rounded up)
  // 如果更近的话就需要扩展得更频繁，
  // 取max是因为如果足够近，那么需要每次迭代都进行扩展
  int desired_iterations =
      std::max(static_cast<int>(closest_distance / _analytic_expansion_ratio), static_cast<int>(std::ceil(_analytic_expansion_ratio)));

  // If we are closer now, we should update the target number of iterations to go
  analytic_iterations = std::min(analytic_iterations, desired_iterations);

  // Always run the expansion on the first run in case there is a
  // trivial path to be found
  if (analytic_iterations <= 0) {
    // Reset the counter and try the analytic path expansion
    analytic_iterations = desired_iterations;
    AnalyticExpansionNodes analytic_nodes = getAnalyticPath(current_node, getter);
    if (!analytic_nodes.empty()) {
      // If we have a valid path, attempt to refine it
      NodePtr node = current_node;
      // TODO
      // NodePtr test_node = current_node;
      // AnalyticExpansionNodes refined_analytic_nodes;
      // for (int i = 0; i < 8; i++) {
      //     // Attempt to create better paths in 5 node increments, need to make sure
      //     // they exist for each in order to do so (maximum of 40 points back).
      //     if (test_node->parent && test_node->parent->parent && test_node->parent->parent->parent &&
      //         test_node->parent->parent->parent->parent && test_node->parent->parent->parent->parent->parent) {
      //         test_node = test_node->parent->parent->parent->parent->parent;
      //         refined_analytic_nodes = getAnalyticPath(test_node, getter);
      //         if (refined_analytic_nodes.empty()) {
      //             break;
      //         }
      //         analytic_nodes = refined_analytic_nodes;
      //         node = test_node;
      //     } else {
      //         break;
      //     }
      // }

      return setAnalyticPath(node, analytic_nodes);
    }
  }

  analytic_iterations--;

  // No valid motion model - return nullptr
  return NodePtr(nullptr);
}

AStarAlgorithm::AnalyticExpansionNodes AStarAlgorithm::getAnalyticPath(const NodePtr& node, const NodeGetter& node_getter) {
  static ompl::base::ScopedState<> from(node->motion_table.state_space), to(node->motion_table.state_space), s(node->motion_table.state_space);
  from[0] = node->pose.x;
  from[1] = node->pose.y;
  from[2] = node->pose.theta * node->motion_table.bin_size;
  to[0] = _goal_coordinates.x;
  to[1] = _goal_coordinates.y;
  to[2] = _goal_coordinates.theta * node->motion_table.bin_size;

  float d = node->motion_table.state_space->distance(from(), to());

  // 距离设为0.5m
  unsigned int num_intervals = std::floor(d / (0.5 / _grid_map->info.resolution));

  AnalyticExpansionNodes possible_nodes;
  // When "from" and "to" are zero or one cell away,
  // num_intervals == 0
  possible_nodes.reserve(num_intervals); // We won't store this node or the goal
  std::vector<double> reals;

  // Pre-allocate
  NodePtr prev(node);
  unsigned int index = 0;
  NodePtr next(nullptr);
  float angle = 0.0;
  Coordinates proposed_coordinates;
  bool failure = false;

  // Check intermediary poses (non-goal, non-start)
  for (float i = 1; i < num_intervals; i++) {
    node->motion_table.state_space->interpolate(from(), to(), i / num_intervals, s());
    reals = s.reals();
    angle = reals[2] / node->motion_table.bin_size;
    while (angle < 0.0) {
      angle += node->motion_table.num_angle_quantization_float;
    }
    while (angle >= node->motion_table.num_angle_quantization_float) {
      angle -= node->motion_table.num_angle_quantization_float;
    }
    // Turn the pose into a node, and check if it is valid
    index = NodeHybrid::getIndex(static_cast<unsigned int>(reals[0]), static_cast<unsigned int>(reals[1]), static_cast<unsigned int>(angle));
    // Get the node from the graph
    if (node_getter(index, next)) {
      Coordinates initial_node_coords = next->pose;
      proposed_coordinates = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
      next->setPose(proposed_coordinates);
      if (next->isNodeValid(_grid_map) && next != prev) {
        // Save the node, and its previous coordinates in case we need to abort
        possible_nodes.emplace_back(next, initial_node_coords, proposed_coordinates);
        prev = next;
      } else {
        // Abort
        next->setPose(initial_node_coords);
        failure = true;
        break;
      }
    } else {
      // Abort
      failure = true;
      break;
    }
  }

  // Reset to initial poses to not impact future searches
  for (const auto& node_pose : possible_nodes) {
    const auto& n = node_pose.node;
    n->setPose(node_pose.initial_coords);
  }

  if (failure) {
    return AnalyticExpansionNodes();
  }

  return possible_nodes;
}

AStarAlgorithm::NodePtr AStarAlgorithm::setAnalyticPath(const NodePtr& node, const AnalyticExpansionNodes& expanded_nodes) {
  // Legitimate final path - set the parent relationships & poses
  NodePtr prev = node;
  for (const auto& node_pose : expanded_nodes) {
    const auto& n = node_pose.node;
    if (!n->wasVisited() && n->getIndex() != _goal->getIndex()) {
      // Make sure this node has not been visited by the regular algorithm.
      // If it has been, there is the (slight) chance that it is in the path we are expanding
      // from, so we should skip it.
      // Skipping to the next node will still create a kinematically feasible path.
      n->parent = prev;
      n->pose = node_pose.proposed_coords;
      n->visited();
      prev = n;
    }
  }
  if (_goal != prev) {
    _goal->parent = prev;
    _goal->visited();
  }
  return _goal;
}

bool AStarAlgorithm::WorldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
  if (wx < 0 || wy < 0) {
    return false;
  }

  mx = static_cast<unsigned int>(wx / _grid_map->info.resolution);
  my = static_cast<unsigned int>(wy / _grid_map->info.resolution);

  if (mx < _grid_map->info.width && my < _grid_map->info.height) {
    return true;
  }
  return false;
}
bool AStarAlgorithm::MapToWorld(float mx, float my, double& wx, double& wy) {
  wx = static_cast<double>(mx * _grid_map->info.resolution);
  wy = static_cast<double>(my * _grid_map->info.resolution);
  return true;
}

inline geometry_msgs::Quaternion getWorldOrientation(const float& theta, const float& bin_size) {
  // theta is in continuous bin coordinates, must convert to world orientation
  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta * static_cast<double>(bin_size));
  return tf2::toMsg(q);
}

nav_msgs::Path HybridASearch(geometry_msgs::Pose start, geometry_msgs::Pose goal, float turn_radius, nav_msgs::OccupancyGridPtr grid_map) {
  std::unique_ptr<AStarAlgorithm> a_star = std::make_unique<AStarAlgorithm>();
  float min_turning_radius = turn_radius;
  unsigned int dim3_size = 72;
  a_star->initialize(false, 100000, 0, dim3_size, grid_map, min_turning_radius / grid_map->info.resolution);
  // 设定起始点
  unsigned int mx, my;
  a_star->WorldToMap(start.position.x, start.position.y, mx, my);
  float orientation_bin = tf2::getYaw(start.orientation) / (2 * M_PI / dim3_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(dim3_size);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(dim3_size)) {
    orientation_bin -= static_cast<float>(dim3_size);
  }
  float wx_in_map = start.position.x / grid_map->info.resolution;
  float wy_in_map = start.position.y / grid_map->info.resolution;
  a_star->setStart(mx, my, orientation_bin, wx_in_map, wy_in_map);

  // 设定目标点
  orientation_bin = tf2::getYaw(goal.orientation) / (2 * M_PI / dim3_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(dim3_size);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(dim3_size)) {
    orientation_bin -= static_cast<float>(dim3_size);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  a_star->WorldToMap(goal.position.x, goal.position.y, mx, my);
  wx_in_map = goal.position.x / grid_map->info.resolution;
  wy_in_map = goal.position.y / grid_map->info.resolution;
  a_star->setGoal(mx, my, orientation_bin_id, wx_in_map, wy_in_map);
  // a_star->setGoal(goal.position.x, goal.position.y, orientation_bin_id);

  std::vector<NodeHybrid::Coordinates> path;
  std::string error;
  nav_msgs::Path plan;
  int num_iterations = 0;
  try {
    // ROS_INFO("Begin to search path");
    if (!a_star->createPath(path, num_iterations, 0.0)) {
      if (num_iterations < a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error& e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    ROS_WARN("Failed to create plan, %s. iterations: %d", error.c_str(), num_iterations);
    return plan;
  }

  plan.header.frame_id = "world";
  plan.header.stamp = ros::Time::now();
  plan.poses.reserve(path.size() + 1);
  geometry_msgs::PoseStamped pose;
  pose.header = plan.header;
  pose.pose = start;
  plan.poses.emplace_back(pose);
  for (int i = path.size() - 1; i >= 0; --i) {
    a_star->MapToWorld(path[i].x, path[i].y, pose.pose.position.x, pose.pose.position.y);
    // pose.pose.position.x = path[i].x;
    // pose.pose.position.y = path[i].y;
    pose.pose.orientation = getWorldOrientation(path[i].theta, 2 * M_PI / dim3_size);
    plan.poses.emplace_back(pose);
  }
  // ROS_INFO("Path has published:%ld", plan.poses.size());
  return plan;
}