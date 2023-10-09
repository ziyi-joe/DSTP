#ifndef __NODE_HYBRID3D__
#define __NODE_HYBRID3D__

#include <vector>

class Node3d {
public:
  Node3d(double x, double y, double yaw)
      : x_(x), y_(y), yaw_(yaw), x_grid_(x / xy_resolution_), y_grid_(y / xy_resolution_), yaw_grid_(yaw / yaw_resolution_) {
    index_ = x * y_size_ * yaw_size_ + y * yaw_size_ + yaw;
  }
  virtual ~Node3d() = default;
  double GetCost() const { return traj_cost_ + heuristic_cost_; }
  double GetTrajCost() const { return traj_cost_; }
  double GetHeuCost() const { return heuristic_cost_; }
  int GetGridX() const { return x_grid_; }
  int GetGridY() const { return y_grid_; }
  int GetGridYaw() const { return yaw_grid_; }
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetYaw() const { return yaw_; }
  bool operator==(const Node3d& right) const;
  int GetIndex() const { return index_; }
  bool GetDirec() const { return direction_; }
  double GetSteer() const { return steering_; }
  Node3d* GetPreNode() const { return pre_node_; }
  void SetPre(Node3d* pre_node) { pre_node_ = pre_node; }
  void SetDirec(bool direction) { direction_ = direction; }
  void SetTrajCost(double cost) { traj_cost_ = cost; }
  void SetHeuCost(double cost) { heuristic_cost_ = cost; }
  void SetSteer(double steering) { steering_ = steering; }
  static int y_size_;
  static int yaw_size_;
  static double xy_resolution_;
  static double yaw_resolution_;

private:
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  int x_grid_ = 0;
  int y_grid_ = 0;
  int yaw_grid_ = 0;
  int index_ = 0;
  double traj_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  double cost_ = 0.0;
  Node3d* pre_node_ = nullptr;
  double steering_ = 0.0;
  // true for moving forward and false for moving backward
  bool direction_ = true;
};

#endif