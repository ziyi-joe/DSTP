#include <ompl/base/ScopedState.h>

#include "path_searching/node_hybrid.h"

// defining static member for all instance to share
std::vector<float> NodeHybrid::obstacle_heuristic_lookup_table;
std::queue<unsigned int> NodeHybrid::obstacle_heuristic_queue;
float NodeHybrid::travel_distance_cost = sqrt(2);
HybridMotionTable NodeHybrid::motion_table;
float NodeHybrid::size_lookup = 25;
std::vector<float> NodeHybrid::dist_heuristic_lookup_table;
bool NodeHybrid::traverse_unknown_; //是否允许扩展到未知节点

void HybridMotionTable::initReedsShepp(unsigned int &size_x_in, unsigned int &size_y_in, unsigned int &angle_quantization_in,
                                       float min_turning_radius, float res) {
  size_x = size_x_in;
  change_penalty = 1.2;
  non_straight_penalty = 1.1;
  cost_penalty = 1.0;
  reverse_penalty = 2.5;

  // if nothing changed, no need to re-compute primitives

  num_angle_quantization = angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius_ = min_turning_radius;

  float angle = 2.0 * asin(0.707 / res / (2 * min_turning_radius)); // set the step
  bin_size = 2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  float delta_x = min_turning_radius * sin(angle);
  float delta_y = min_turning_radius - (min_turning_radius * cos(angle));

  projections.clear();
  projections.reserve(6);
  projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
  projections.emplace_back(delta_x, delta_y, increments);        // Forward + Left
  projections.emplace_back(delta_x, -delta_y, -increments);      // Forward + Right
  projections.emplace_back(-hypotf(delta_x, delta_y), 0.0, 0.0); // Backward
  projections.emplace_back(-delta_x, delta_y, -increments);      // Backward + Left
  projections.emplace_back(-delta_x, -delta_y, increments);      // Backward + Right

  // Create the correct OMPL state space
  state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(min_turning_radius);

  // 提前计算好不同角度下的6种运动方式的delta_x和delta_y，也就意味着这是世界坐标系下的
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        // if first iteration, cache the trig values for later
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i][0] * cos_theta - projections[i][1] * sin_theta;
      delta_ys[i][j] = projections[i][0] * sin_theta + projections[i][1] * cos_theta;
    }
  }
}

std::vector<MotionPose> HybridMotionTable::getProjections(const NodeHybrid *node) {
  std::vector<MotionPose> projection_list;
  projection_list.reserve(projections.size());

  for (unsigned int i = 0; i != projections.size(); i++) {
    const MotionPose &motion_model = projections[i];

    // normalize theta, I know its overkill, but I've been burned before...
    const float &node_heading = node->pose.theta;
    float new_heading = node_heading + motion_model(2);

    if (new_heading < 0.0) {
      new_heading += num_angle_quantization_float;
    }

    if (new_heading >= num_angle_quantization_float) {
      new_heading -= num_angle_quantization_float;
    }

    projection_list.emplace_back(delta_xs[i][node_heading] + node->pose.x, delta_ys[i][node_heading] + node->pose.y, new_heading);
  }

  return projection_list;
}

/**
 * @brief 通过节点的下标来进行初始化
 *
 * @param index
 */
NodeHybrid::NodeHybrid(const unsigned int index)
    : parent(nullptr)
    , pose(0.0f, 0.0f, 0.0f)
    , graph_node_ptr(nullptr)
    , _cell_cost(std::numeric_limits<float>::quiet_NaN())
    , _accumulated_cost(std::numeric_limits<float>::max())
    , _index(index)
    , _was_visited(false)
    , _motion_primitive_index(std::numeric_limits<unsigned int>::max()) {}

NodeHybrid::~NodeHybrid() { parent = nullptr; }

bool NodeHybrid::isNodeValid(nav_msgs::OccupancyGridPtr grid_map) {
  // TODO 需要修改_cell_cost
  int idx = std::round(pose.x), idy = std::round(pose.y);
  // int idx = std::round(pose.x / grid_map->info.resolution), idy = std::round(pose.y / grid_map->info.resolution);
  if (!traverse_unknown_) {
    if (idx >= static_cast<int>(grid_map->info.width) || idy >= static_cast<int>(grid_map->info.height)) return false;
    if (idx < 0 || idy < 0) return false;
    if (grid_map->data[idx + idy * grid_map->info.width] < 50) {
      _cell_cost = 0;
      return true;
    } else
      return false;
  } else {
    if (grid_map->data[idx + idy * grid_map->info.width] < 50) {
      _cell_cost = 0;
      return true;
    } else
      return false;
  }
}

float NodeHybrid::getTraversalCost(const NodePtr &child) {
  const float normalized_cost = child->getCost() / 252.0;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
        "Node attempted to get traversal "
        "cost without a known SE2 collision cost!");
  }

  // 如果是起始第一个节点
  if (getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max()) {
    return NodeHybrid::travel_distance_cost;
  }

  // Note(stevemacenski): `travel_cost_raw` at one point contained a term:
  // `+ motion_table.cost_penalty * normalized_cost;`
  // It has been removed, but we may want to readdress this point and determine
  // the technically and theoretically correctness of that choice. I feel technically speaking
  // that term has merit, but it doesn't seem to impact performance or path quality.
  // W/o it lowers the travel cost, which would drive the heuristics up proportionally where I
  // would expect it to plan much faster in all cases, but I only see it in some cases. Since
  // this term would weight against moving to high cost zones, I would expect to see more smooth
  // central motion, but I only see it in some cases, possibly because the obstacle heuristic is
  // already driving the robot away from high cost areas; implicitly handling this. However,
  // then I would expect that not adding it here would make it unbalanced enough that path quality
  // would suffer, which I did not see in my limited experimentation, possibly due to the smoother.
  float travel_cost = 0.0;
  float travel_cost_raw = NodeHybrid::travel_distance_cost;

  if (child->getMotionPrimitiveIndex() == 0 || child->getMotionPrimitiveIndex() == 3) {
    // 新的动作是直线，则不增加惩罚（倒车惩罚在后面）
    travel_cost = travel_cost_raw;
  } else {
    if (getMotionPrimitiveIndex() == child->getMotionPrimitiveIndex()) {
      // 转向，但是和上一次是同方向的转向
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    } else {
      travel_cost = travel_cost_raw * (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  if (getMotionPrimitiveIndex() > 2) {
    // 倒车惩罚
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeHybrid::getHeuristicCost(const Coordinates &node_coords, const Coordinates &goal_coords, const nav_msgs::OccupancyGridPtr grid_map) {
  const float obstacle_heuristic = getObstacleHeuristic(node_coords, goal_coords, grid_map);
  const float dist_heuristic = getDistanceHeuristic(node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, dist_heuristic);
}

void NodeHybrid::initMotionModel(unsigned int &size_x, unsigned int &size_y, unsigned int &num_angle_quantization, float min_turning_radius,
                                 float res) {
  motion_table.initReedsShepp(size_x, size_y, num_angle_quantization, min_turning_radius, res);
  travel_distance_cost = motion_table.projections[0](0);
}

void NodeHybrid::resetObstacleHeuristic(nav_msgs::OccupancyGridPtr grid_map, const unsigned int &goal_x, const unsigned int &goal_y) {
  // TODO
  unsigned int size = grid_map->info.height * grid_map->info.width;
  if (obstacle_heuristic_lookup_table.size() == size) {
    // 如果地图大小没变化，就直接全部置0
    std::fill(obstacle_heuristic_lookup_table.begin(), obstacle_heuristic_lookup_table.end(), 0.0);
  } else {
    // 这三步将obstacle_heuristic_lookup_table resize成和sampled_costmap一样大小，并且全部附0
    unsigned int obstacle_size = obstacle_heuristic_lookup_table.size();
    obstacle_heuristic_lookup_table.resize(size, 0.0);
    std::fill_n(obstacle_heuristic_lookup_table.begin(), obstacle_size, 0.0); // 因为后面都是resize出来的，本来就是0，所以只需要把前面的重新归0
  }

  // 将终点加入queue里
  std::queue<unsigned int> q;
  std::swap(obstacle_heuristic_queue, q);
  obstacle_heuristic_queue.emplace(goal_y * grid_map->info.width + goal_x); // ui型的还有必要ceil吗
}

float NodeHybrid::getObstacleHeuristic(const Coordinates &node_coords, const Coordinates &goal_coords, nav_msgs::OccupancyGridPtr grid_map) {
  // TODO
  unsigned int size_x = grid_map->info.width;
  const unsigned int start_index = ceil(node_coords.y) * size_x + ceil(node_coords.x); //当前位置
  const float &starting_cost = obstacle_heuristic_lookup_table[start_index];
  if (starting_cost > 0.0) { // 如果已经被扩展过，就直接返回cost
    return starting_cost;
  }

  // If not, expand until it is included. This dynamic programming ensures that
  // we only expand the MINIMUM spanning set of the costmap per planning request.
  // Rather than naively expanding the entire (potentially massive) map for a limited
  // path, we only expand to the extent required for the furthest expansion in the
  // search-planning request that dynamically updates during search as needed.
  // 如果没有，就扩展直到该节点
  // 采用动态规划的方式，保证每个节点只扩展一次
  const int size_x_int = static_cast<int>(size_x);
  const unsigned int size_y = grid_map->info.height;
  const float sqrt_2 = sqrt(2);
  unsigned int mx, my, mx_idx, my_idx;
  unsigned int idx = 0, new_idx = 0;
  float last_accumulated_cost = 0.0, travel_cost = 0.0;
  float current_accumulated_cost = 0.0;
  float cost = 0.0, existing_cost = 0.0;

  const std::vector<int> neighborhood = {1,
                                         -1, // left right
                                         size_x_int,
                                         -size_x_int, // up down
                                         size_x_int + 1,
                                         size_x_int - 1, // upper diagonals
                                         -size_x_int + 1,
                                         -size_x_int - 1}; // lower diagonals

  while (!obstacle_heuristic_queue.empty()) {
    // get next one
    idx = obstacle_heuristic_queue.front();
    obstacle_heuristic_queue.pop();
    last_accumulated_cost = obstacle_heuristic_lookup_table[idx];

    if (idx == start_index) { // 如果遍历到了当前位置
      return last_accumulated_cost;
    }

    my_idx = idx / size_x;
    mx_idx = idx - (my_idx * size_x);

    // 寻找邻居
    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);
      cost = static_cast<float>(grid_map->data[idx]);
      travel_cost = ((i <= 3) ? 1.0 : sqrt_2) + (motion_table.cost_penalty * cost / 100.0); //距离cost+成本地图cost
      current_accumulated_cost = last_accumulated_cost + travel_cost;

      // if neighbor path is better and non-lethal, set new cost and add to queue
      // 如果邻居的路径更好且不和障碍物碰撞
      if (new_idx > 0 && new_idx < size_x * size_y && cost < 50.0) {
        my = new_idx / size_x; // map坐标系下的y
        mx = new_idx - (my * size_x);

        // 如果跨行了，比如[2,10]到了[3,0]
        if ((mx == 0 && mx_idx >= size_x - 1) || (mx >= size_x - 1 && mx_idx == 0)) {
          continue;
        }
        if ((my == 0 && my_idx >= size_y - 1) || (my >= size_y - 1 && my_idx == 0)) {
          continue;
        }

        existing_cost = obstacle_heuristic_lookup_table[new_idx];
        // 如果当前邻居节点的障碍启发式函数未分配 或者 大于经过当前节点到达的cost
        if (existing_cost == 0.0 || existing_cost > current_accumulated_cost) {
          obstacle_heuristic_lookup_table[new_idx] = current_accumulated_cost;
          obstacle_heuristic_queue.emplace(new_idx);
        }
      }
    }
  }

  return obstacle_heuristic_lookup_table[start_index];
}

float NodeHybrid::getDistanceHeuristic(const Coordinates &node_coords, const Coordinates &goal_coords, const float &obstacle_heuristic) {
  // TODO
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // Due to the rounding involved in exact cell increments for caching,
  // this is not an exact replica of a live heuristic, but has bounded error.
  // (Usually less than 1 cell)
  // 利用旋转平移来使终点的坐标为(0,0,0)
  // 因为之前用到的近似，所以这里会有一点误差，一般都在一个cell内

  // This angle is negative since we are de-rotating the current node
  // by the goal angle; cos(-th) = cos(th) & sin(-th) = -sin(th)
  //
  const TrigValues &trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  } else if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(round(dx * cos_th - dy * sin_th), round(dx * sin_th + dy * cos_th), round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table is contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  float motion_heuristic = 0.0;
  const int floored_size = floor(size_lookup / 2.0);
  const int ceiling_size = ceil(size_lookup / 2.0);
  const float mirrored_relative_y = abs(node_coords_relative.y);
  if (abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {
    // Need to mirror angle if Y coordinate was mirrored
    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index = x_pos * ceiling_size * motion_table.num_angle_quantization + y_pos * motion_table.num_angle_quantization + theta_pos;
    motion_heuristic = dist_heuristic_lookup_table[index];
  } else if (obstacle_heuristic == 0.0) {
    // If no obstacle heuristic value, must have some H to use
    // In nominal situations, this should never be called.
    static ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    to[2] = goal_coords.theta * motion_table.num_angle_quantization;
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = node_coords.theta * motion_table.num_angle_quantization;
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

// 提前计算好R-S扩展的距离启发值
void NodeHybrid::precomputeDistanceHeuristic(const float &lookup_table_dim, const unsigned int &dim_3_size, const float &min_turning_radius) {
  // Dubin or Reeds-Shepp shortest distances

  motion_table.state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(min_turning_radius);

  ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);
  float angular_bin_size = 2 * M_PI / static_cast<float>(dim_3_size);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Due to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  // 只需要储存4个象限中的两个，另外的可以关于X轴反转
  dist_heuristic_lookup_table.resize(size_lookup * ceil(size_lookup / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = heading * angular_bin_size;
        motion_heuristic = motion_table.state_space->distance(from(), to());
        dist_heuristic_lookup_table[index] = motion_heuristic;
        index++;
      }
    }
  }
}

void NodeHybrid::getNeighbors(std::function<bool(const unsigned int &, NodeHybrid *&)> &NeighborGetter, const bool &traverse_unknown,
                              NodeVector &neighbors, nav_msgs::OccupancyGridPtr grid_map) {
  unsigned int index = 0;
  NodeHybrid *neighbor = nullptr;
  Coordinates initial_node_coords;
  const std::vector<MotionPose> motion_projections = motion_table.getProjections(this);

  // 利用运动模型来检索邻居节点，R-S曲线会有6种
  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeHybrid::getIndex(static_cast<unsigned int>(motion_projections[i](0)), static_cast<unsigned int>(motion_projections[i](1)),
                                 static_cast<unsigned int>(motion_projections[i](2)));

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion 不希望中断连续坐标扩展
      // 相当于更新了该index代表的node的pose，但有可能更新后是invalid
      initial_node_coords = neighbor->pose;
      neighbor->setPose(Coordinates(motion_projections[i](0), motion_projections[i](1), motion_projections[i](2)));
      if (neighbor->isNodeValid(grid_map)) {
        neighbor->setMotionPrimitiveIndex(i); // 每个节点的MotionPrimitiveIndex都是上个节点到当前节点的运动方式
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}