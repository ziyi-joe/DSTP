#ifndef __NODE_HYBRID_HPP__
#define __NODE_HYBRID_HPP__

#include <Eigen/Core>
#include <vector>
#include <queue>

#include <ompl-1.5/ompl/base/spaces/ReedsSheppStateSpace.h>
#include <nav_msgs/OccupancyGrid.h>

typedef Eigen::Vector3f MotionPose;
typedef std::pair<double, double> TrigValues;

// Forward declare
class NodeHybrid;

/**
 * @struct nav2_smac_planner::HybridMotionTable
 * @brief R-S运动的一些参数
 */
struct HybridMotionTable {
  HybridMotionTable() {}

  void initReedsShepp(unsigned int &size_x_in, unsigned int &size_y_in, unsigned int &angle_quantization_in, float min_turning_radius, float res);

  /**
   * @brief 通过当前节点扩展得到6个新的MotionPose
   */
  std::vector<MotionPose> getProjections(const NodeHybrid *node);

  std::vector<MotionPose> projections;
  unsigned int size_x;
  unsigned int num_angle_quantization;
  float num_angle_quantization_float;
  float min_turning_radius_;
  float bin_size;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  ompl::base::StateSpacePtr state_space;
  std::vector<std::vector<float>> delta_xs;
  std::vector<std::vector<float>> delta_ys;
  std::vector<TrigValues> trig_values;
};

class NodeHybrid {
public:
  typedef NodeHybrid *NodePtr;
  typedef std::unique_ptr<std::vector<NodeHybrid>> Graph;
  typedef std::vector<NodePtr> NodeVector;

  /**
   * @brief NodeHybrid 节点的坐标
   */
  struct Coordinates {
    Coordinates() {}

    Coordinates(const float &x_in, const float &y_in, const float &theta_in) : x(x_in), y(y_in), theta(theta_in) {}

    inline bool operator==(const Coordinates &rhs) { return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta; }

    inline bool operator!=(const Coordinates &rhs) { return !(*this == rhs); }

    float x, y, theta;
  };

  NodeHybrid(const unsigned int index);

  ~NodeHybrid();

  bool operator==(const NodeHybrid &rhs) { return this->_index == rhs._index; }

  inline void setPose(const Coordinates &pose_in) { pose = pose_in; }

  /**
   * @brief Reset method for new search
   */
  void reset();

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  inline float &getAccumulatedCost() { return _accumulated_cost; }

  inline void setAccumulatedCost(const float &cost_in) { _accumulated_cost = cost_in; }

  /**
   * @brief 设定到达该节点的运动方式，有6种
   */
  inline void setMotionPrimitiveIndex(const unsigned int &idx) { _motion_primitive_index = idx; }

  inline unsigned int getMotionPrimitiveIndex() const { return _motion_primitive_index; }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  inline float getCost() const { return _cell_cost; }

  inline bool &wasVisited() { return _was_visited; }

  inline void visited() { _was_visited = true; }

  inline unsigned int &getIndex() { return _index; }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown 是否可以探索未知即节点
   * @return whether this node is valid and collision free
   */
  bool isNodeValid(nav_msgs::OccupancyGridPtr grid_map);

  /**
   * @brief 得到父节点到子结点的traversal cost
   * @param child Node pointer to child
   * @return traversal cost
   */
  float getTraversalCost(const NodePtr &child);

  /**
   * @brief 是static的，通过坐标来计算某个节点的index
   * @param x X coordinate of point
   * @param y Y coordinate of point
   * @param angle Theta coordinate of point
   * @param width Width of costmap
   * @param angle_quantization Number of theta bins
   * @return Index
   */
  static inline unsigned int getIndex(const unsigned int &x, const unsigned int &y, const unsigned int &angle) {
    return angle + x * motion_table.num_angle_quantization + y * motion_table.size_x * motion_table.num_angle_quantization;
  }

  /**
   * @brief 通过index得到坐标Coordinates
   * @param index Index of point
   * @param width Width of costmap
   * @param angle_quantization Theta size of costmap
   * @return Coordinates
   */
  static inline Coordinates getCoords(const unsigned int &index, const unsigned int &width, const unsigned int &angle_quantization) {
    return Coordinates((index / angle_quantization) % width, // x
                       index / (angle_quantization * width), // y
                       index % angle_quantization);          // theta
  }

  static float getHeuristicCost(const Coordinates &node_coords, const Coordinates &goal_coordinates, const nav_msgs::OccupancyGridPtr grid_map);

  /**
   * @brief Initialize motion models
   * @param motion_model Motion model enum to use
   * @param size_x Size of X of graph
   * @param size_y Size of y of graph
   * @param angle_quantization Size of theta bins of graph
   * @param search_info Search info to use
   */
  static void initMotionModel(unsigned int &size_x, unsigned int &size_y, unsigned int &angle_quantization, float min_turning_radius, float rs);

  /**
   * @brief Compute the SE2 distance heuristic
   * @param lookup_table_dim Size, in costmap pixels, of the
   * each lookup table dimension to populate
   * @param dim_3_size Number of quantization bins for caching
   * @param search_info Info containing minimum radius to use
   */
  static void precomputeDistanceHeuristic(const float &lookup_table_dim, const unsigned int &dim_3_size, const float &min_turning_radius);

  /**
   * @brief Compute the Obstacle heuristic,只考虑障碍物信息而不考虑车辆的非完整性约束条件
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @return heuristic Heuristic value
   */
  static float getObstacleHeuristic(const Coordinates &node_coords, const Coordinates &goal_coords, nav_msgs::OccupancyGridPtr grid_map);

  /**
   * @brief Compute the Distance heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @param obstacle_heuristic Value of the obstacle heuristic to compute
   * additional motion heuristics if required
   * @return heuristic Heuristic value
   */
  static float getDistanceHeuristic(const Coordinates &node_coords, const Coordinates &goal_coords, const float &obstacle_heuristic);

  /**
   * @brief 将障碍物启发函数的table设置为grid_map同样的大小并全部置0
   * @param costmap Costmap to use
   * @param goal_coords Coordinates to start heuristic expansion at
   */
  static void resetObstacleHeuristic(nav_msgs::OccupancyGridPtr grid_map, const unsigned int &goal_x, const unsigned int &goal_y);

  /**
   * @brief 检索某节点所有有效的邻居
   * @param validity_checker Functor for state validity checking
   * @param collision_checker Collision checker to use
   * @param traverse_unknown If unknown costs are valid to traverse
   * @param neighbors Vector of neighbors to be filled
   */
  void getNeighbors(std::function<bool(const unsigned int &, NodeHybrid *&)> &validity_checker, const bool &traverse_unknown, NodeVector &neighbors,
                    nav_msgs::OccupancyGridPtr grid_map);

  NodeHybrid *parent;
  Coordinates pose;           // 是地图坐标系下的node，但角度是bin下的
  NodeHybrid *graph_node_ptr; //指向自己的指针
  // TODO 希望取消掉这个指针，主要是queue里面会用到

  static float travel_distance_cost; // 从一个节点到另一个节点的基本cost
  static HybridMotionTable motion_table;
  // Wavefront lookup and queue for continuing to expand as needed
  static std::vector<float> obstacle_heuristic_lookup_table;
  static std::queue<unsigned int> obstacle_heuristic_queue; // 等待扩展的队列
  // Dubin / Reeds-Shepp lookup and size for dereferencing
  static std::vector<float> dist_heuristic_lookup_table;
  static float size_lookup;
  static bool traverse_unknown_; //是否允许扩展到未知节点

private:
  float _cell_cost;
  float _accumulated_cost; //起点到该点的cost(g_cost)
  unsigned int _index;
  bool _was_visited;
  unsigned int _motion_primitive_index;
};

#endif