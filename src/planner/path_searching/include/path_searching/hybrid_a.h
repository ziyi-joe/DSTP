#ifndef __HYBRID_A_HPP__
#define __HYBRID_A_HPP__

#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>
#include "Eigen/Core"
#include <tf/transform_listener.h>

#include "path_searching/node_hybrid.h"

typedef std::pair<float, unsigned int> NodeHeuristicPair;

/**
 * @class nav2_smac_planner::AStarAlgorithm
 * @brief An A* implementation for planning in a costmap. Templated based on the Node type.
 */
class AStarAlgorithm {
  public:
  typedef NodeHybrid *NodePtr;
  typedef std::pair<float, NodeHybrid> NodeElement;
  typedef typename NodeHybrid::Coordinates Coordinates;
  typedef std::vector<NodePtr>::iterator NeighborIterator;
  typedef std::function<bool(const unsigned int &, NodePtr &)> NodeGetter;

  /**
   * @struct nav2_smac_planner::AnalyticExpansionNodes
   * @brief Analytic expansion nodes and associated metadata
   */
  struct AnalyticExpansionNode {
    AnalyticExpansionNode(NodePtr &node_in, Coordinates &initial_coords_in, Coordinates &proposed_coords_in)
        : node(node_in), initial_coords(initial_coords_in), proposed_coords(proposed_coords_in) {}

    NodePtr node;
    Coordinates initial_coords;
    Coordinates proposed_coords;
  };

  typedef std::vector<AnalyticExpansionNode> AnalyticExpansionNodes;

  /**
   * @struct nav2_smac_planner::NodeComparator
   * @brief Node comparison for priority queue sorting
   */
  struct NodeComparator {
    bool operator()(const NodeElement &a, const NodeElement &b) const { return a.first > b.first; }
  };

  typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

  /**
   * @brief A constructor for nav2_smac_planner::PlannerServer
   * @param neighborhood The type of neighborhood to use for search (4 or 8 connected)
   */
  explicit AStarAlgorithm();

  /**
   * @brief A destructor for nav2_smac_planner::AStarAlgorithm
   */
  ~AStarAlgorithm();

  /**
   * @brief Initialization of the planner with defaults
   * @param allow_unknown 是否允许扩展到未知区域
   * @param max_iterations Maximum number of iterations to use while expanding search
   * @param dim_3_size 角度被分为多少份
   */
  void initialize(const bool allow_unknown, int max_iterations, const float lookup_table_size, const unsigned int dim_3_size,
                  nav_msgs::OccupancyGridPtr grid_map, float min_turning_radius);

  /**
   * @brief Creating path from given costmap, start, and goal
   * @param path Reference to a vector of indicies of generated path
   * @param num_iterations Reference to number of iterations to create plan
   * @param tolerance Reference to tolerance in costmap nodes
   * @return if plan was successful
   */
  bool createPath(std::vector<Coordinates> &path, int &num_iterations, const float &tolerance);

  /**
   * @brief 通过x,y,theta的index来设定终点
   */
  void setGoal(const unsigned int &mx, const unsigned int &my, const unsigned int &dim_3, const float wx_in_map, const float wy_in_map);

  /**
   * @brief 通过x,y,theta的index来设定起点
   */
  void setStart(const unsigned int &mx, const unsigned int &my, const unsigned int &dim_3, const float wx_in_map, const float wy_in_map);

  /**
   * @brief Get maximum number of iterations to plan
   * @return Reference to Maximum iterations parameter
   */
  int &getMaxIterations();

  /**
   * @brief Get pointer reference to starting node
   * @return Node pointer reference to starting node
   */
  NodePtr &getStart();

  /**
   * @brief Get pointer reference to goal node
   * @return Node pointer reference to goal node
   */
  NodePtr &getGoal();

  int &getOnApproachMaxIterations();

  /**
   * @brief Get tolerance, in node nodes
   * @return Reference to tolerance parameter
   */
  float &getToleranceHeuristic();

  unsigned int &getSizeX();

  unsigned int &getSizeY();

  /**
   * @brief 返回角度的维数
   */
  unsigned int &getSizeDim3();

  bool WorldToMap(double wx, double wy, unsigned int &mx, unsigned int &my);
  bool MapToWorld(float mx, float my, double &wx, double &wy);

  private:
  /**
   * @brief Get pointer to next goal in open set
   * @return Node pointer reference to next heuristically scored node
   */
  inline NodePtr getNextNode();

  /**
   * @brief 把节点加入到优先级队列openset中
   */
  inline void addNode(const float &cost, NodePtr &node);

  /**
   * @brief 将节点加入graph中，同时返回该节点
   */
  inline NodePtr addToGraph(const unsigned int &index);

  /**
   * @brief 判断节点是否是目标点
   */
  inline bool isGoal(NodePtr &node);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param node Node pointer to the goal node to backtrace
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  bool backtracePath(NodePtr node, std::vector<Coordinates> &path);

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  inline float getHeuristicCost(NodePtr &node);

  /**
   * @brief 输入是否valid
   */
  inline bool areInputsValid();

  /**
   * @brief Clear hueristic queue of nodes to search
   */
  inline void clearQueue();

  /**
   * @brief Clear graph of nodes searched
   */
  inline void clearGraph();

  /**
   * @brief 尝试RS曲线扩展
   * @return Node pointer reference to goal node if successful, else return nullptr
   */
  NodePtr tryAnalyticExpansion(const NodePtr &current_node, const NodeGetter &getter, int &iterations, int &best_cost);

  /**
   * @brief Perform an analytic path expansion to the goal
   * @param node The node to start the analytic path from
   * @param getter The function object that gets valid nodes from the graph
   * @return A set of analytically expanded nodes to the goal from current node, if possible
   */
  AnalyticExpansionNodes getAnalyticPath(const NodePtr &node, const NodeGetter &node_getter);

  /**
   * @brief Takes final analytic expansion and appends to current expanded node
   * @param node The node to start the analytic path from
   * @param expanded_nodes Expanded nodes to append to end of current search path
   * @return Node pointer to goal node if successful, else return nullptr
   */
  NodePtr setAnalyticPath(const NodePtr &node, const AnalyticExpansionNodes &expanded_nodes);

  bool _traverse_unknown = false;
  bool _cache_obstacle_heuristic = true;
  int _max_iterations;
  int _max_on_approach_iterations;
  float _tolerance;
  unsigned int _x_size;
  unsigned int _y_size;
  unsigned int _dim3_size;
  float _analytic_expansion_ratio;

  Coordinates _goal_coordinates;
  NodePtr _start;
  NodePtr _goal;

  std::unordered_map<unsigned int, NodeHybrid> _graph; // index->node的映射
  NodeQueue _queue;

  NodeHeuristicPair _best_heuristic_node; // 按启发式算法计算的最优节点{heuristic, index}

  nav_msgs::OccupancyGridPtr _grid_map;
};

nav_msgs::Path HybridASearch(geometry_msgs::Pose start, geometry_msgs::Pose goal, float turn_radius, nav_msgs::OccupancyGridPtr grid_map);
#endif
