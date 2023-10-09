#ifndef _RPG_PLANNER_H_
#define _RPG_PLANNER_H_

#include <algorithm>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <vector>

#include <string>
#include <unordered_map>

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "optimizer/DSTP_optimizer.h"
#include "dstp_planner/DSTPtraj.h"

namespace dstp_planner {

class RPGPlanner {
public:
  ros::NodeHandle nh;

  RPGPlanner(ros::NodeHandle &nh_) : nh(nh_) {}
  ~RPGPlanner() {}

  void init();

private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* parameters */
  bool flag_have_gridmap_ = false;
  bool flag_have_obs_pointcloud_ = false;
  bool flag_have_map_ = false;
  bool flag_have_odom_ = false;
  bool flag_have_goal_ = false;

  geometry_msgs::Pose odom_pose_, goal_pose_;
  Eigen::VectorXd init_state_, goal_state_; // x,y,yaw,v,kappa,a

  nav_msgs::Path new_rough_path_;  //当前由路径搜索算法得到的路径
  nav_msgs::Path historical_path_; //前一次loop中所实际跟随的路径
  double historical_path_cost_;    //历史路径的代价值
  nav_msgs::Path chosen_path_;     //当前次实际被选中要跟随的路径
  nav_msgs::Path DSTP_path_;       // the path optimized by DSTP
  double chosen_path_cost_;        //当前选择路径的代价值
  double weight_dist_ = 1.0, weight_yaw_ = 1.0;

  int cell_radius_ = 0;

  // trajectory optimization related parameters and functions
  dstp_planner::DSTPtraj DSTP_traj_;
  DSTPOptimizer DSTP_opt_;
  DSTPParameter DSTP_config_;

  /* ROS utils */
  ros::Timer exec_timer_;
  ros::Subscriber odom_sub_, gridmap_sub_, rough_path_sub_, cur_goal_sub_, acc_sub_;
  ros::Publisher updated_gridmap_pub_, obstacle_pointcloud_pub_, cur_safe_corridor_pub_, pruned_path_pub_, DSTP_traj_pub_, DSTP_path_pub_;

  /* state machine functions */
  void execCallback(const ros::TimerEvent &e);

  /* map related parameters */
  // The static map read from map_server
  nav_msgs::OccupancyGrid origin_gridmap_;
  // The edge points
  std::vector<std::pair<int, int>> map_edge_points;
  // The inflated gridmap
  nav_msgs::OccupancyGrid updated_gridmap_;

  //点云形式的所有当前障碍物，所有/静态/动态
  sensor_msgs::PointCloud obs_cloud_total_, obs_cloud_static_, obs_cloud_dynamic_;
  vec_E<Vec2f> obs2d_; // 2d obstacle point for decompROS

  bool get_grid_idx(geometry_msgs::Point &a, int &h, int &w) {
    h = std::round(a.y / origin_gridmap_.info.resolution);
    w = std::round(a.x / origin_gridmap_.info.resolution);
    if (h < 0 || w < 0 || h >= origin_gridmap_.info.height || w >= origin_gridmap_.info.width) return false;
    return true;
  }

  bool checkPathFeasibility(nav_msgs::Path &path) {
    int h, w;
    for (auto &pose : path.poses) {
      if (get_grid_idx(pose.pose.position, h, w)) {
        if (updated_gridmap_.data[h * updated_gridmap_.info.width + w] < 50) {
          continue;
        } else {
          return false;
        }
      }
    }
    return true;
  }

  double calculatePathCost(nav_msgs::Path &path, double weight_dist, double weight_yaw);

  enum GRIDMAP_TYPE { FREE = 0, OCCUPANCIED = 100, OBS_EDGE = 99, DYNAMIC_OBS = 98, OBS_INFLATION = 96 };

  /* map related functions */
  void InflateGridmap();
  bool isEdge(int x, int y);
  // direction = 0表示是顶点，1,2,3,4分别表示是左/上/右/下边界
  void inflateGridPoint(nav_msgs::OccupancyGrid &map, int h, int w, int cell_radius);
  //判断两个pose是否在eps的误差下是相等的，如果相等就返回true，否则返回false
  bool isPoseEqual(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, double p_eps, double rad_eps);
  double getAbsoluteDistance(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b);
  double getPoseDiffCost(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, double weight_dist, double weight_yaw);
  void structCorridor(const nav_msgs::Path &path, std::vector<SafeRegion> *safe_corridor);
  void pruneGlobalPath(nav_msgs::Path &path_);
  bool generateTrajectory(std::vector<SafeRegion> *safe_corridor);

  /* input-output */
  void odometryCallback(const nav_msgs::Odometry &msg);
  void gridmapCallback(const nav_msgs::OccupancyGrid &msg);
  void roughPathCallback(const nav_msgs::Path &msg);
  void goalCallback(geometry_msgs::PoseStamped::ConstPtr msg);
  void accCallback(const std_msgs::Float64 &msg);
};

} // namespace dstp_planner

#endif //_RPG_PLANNER_H_