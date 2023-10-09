#include "plan_manage/rpg_planner.h"

#include <unordered_map>
#include <tf/transform_datatypes.h>

#include "plan_manage/tic_toc.h"

namespace dstp_planner {

void RPGPlanner::init() {
  nh.getParam("MinVel", DSTP_config_.min_vel);
  nh.getParam("MaxVel", DSTP_config_.max_vel);
  nh.getParam("MaxAcc", DSTP_config_.max_acc);
  nh.getParam("MaxKappa", DSTP_config_.max_kappa);
  nh.getParam("SafeRadius", DSTP_config_.safe_radius);
  nh.getParam("FrontWheel", DSTP_config_.wheel_base);

  /* callback */
  exec_timer_ = nh.createTimer(ros::Duration(0.05), &RPGPlanner::execCallback, this);

  odom_sub_ = nh.subscribe("/odom", 1, &RPGPlanner::odometryCallback, this);
  gridmap_sub_ = nh.subscribe("/map", 1, &RPGPlanner::gridmapCallback, this);
  rough_path_sub_ = nh.subscribe("/rough_hybrid_a", 1, &RPGPlanner::roughPathCallback, this);
  cur_goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &RPGPlanner::goalCallback, this);
  acc_sub_ = nh.subscribe("/now_a", 1, &RPGPlanner::accCallback, this);

  updated_gridmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/updated_gridmap", 1);
  obstacle_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("/obstacle_pointcloud", 1);
  cur_safe_corridor_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/cur_safe_corridor_poly", 1);
  pruned_path_pub_ = nh.advertise<nav_msgs::Path>("/chosen_path", 1);
  DSTP_traj_pub_ = nh.advertise<dstp_planner::DSTPtraj>("/DSTP_traj", 1);
  DSTP_path_pub_ = nh.advertise<nav_msgs::Path>("/DSTP_path", 1);

  obs_cloud_total_.header.stamp = ros::Time::now();
  obs_cloud_total_.header.frame_id = "map";
  obs_cloud_total_.channels.resize(1);
  obs_cloud_total_.channels[0].name = "rgb";
  init_state_ = Eigen::VectorXd::Zero(6);
  goal_state_ = Eigen::VectorXd::Zero(5);
}

void RPGPlanner::execCallback(const ros::TimerEvent &e) {
  exec_timer_.stop();
  TicToc total_timer;
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 20) {
    fsm_num = 0;
    updated_gridmap_pub_.publish(updated_gridmap_);
  }
  TicToc corridor_timer;

  // generate trajectory if have global path
  if (flag_have_goal_) {
    double new_path_cost;
    if (new_rough_path_.poses.size() >= 2 && isPoseEqual(new_rough_path_.poses.back().pose, goal_pose_, 0.1, 0.1)) {
      // prune the global path based on the current robot's pose: odom_pose_
      pruneGlobalPath(new_rough_path_);
      new_path_cost = calculatePathCost(new_rough_path_, weight_dist_, weight_yaw_);
      chosen_path_cost_ = new_path_cost;
      chosen_path_ = new_rough_path_;
    }
    if (historical_path_.poses.size() >= 2) {
      // there is historical path
      if (isPoseEqual(historical_path_.poses.back().pose, goal_pose_, 0.1, 0.1)) {
        // the goal does not change, we should compare with the historical path
        pruneGlobalPath(historical_path_);
        if (getAbsoluteDistance(odom_pose_, historical_path_.poses.front().pose) <= 0.5 && checkPathFeasibility(historical_path_)) {
          // make sure the historical path is feasible
          // TODO: update the historical path
          historical_path_cost_ = calculatePathCost(historical_path_, weight_dist_, weight_yaw_);
          if (historical_path_cost_ < chosen_path_cost_) {
            // choose the historical path
            chosen_path_ = historical_path_;
            chosen_path_cost_ = historical_path_cost_;
            // ROS_INFO("Choose historical old path, old cost is %.3f, new cost is %.3f.", historical_path_cost_, new_path_cost);
          } else {
            // ROS_INFO("Choose current new path, old cost is %.3f, new cost is %.3f.", historical_path_cost_, new_path_cost);
          }
        }
      } else {
        // the goal is new
        ROS_INFO("Get a new goal, current cost is %.3f", chosen_path_cost_);
      }
    }
    // chosen_path has been set
    if (chosen_path_.poses.size() >= 3 && isPoseEqual(chosen_path_.poses.back().pose, goal_pose_, 0.1, 0.1)) {
      // construct the safe corridor
      corridor_timer.tic();
      std::vector<SafeRegion> safe_corridor;
      structCorridor(chosen_path_, &safe_corridor);
      auto corridor_time = corridor_timer.toc();
      pruned_path_pub_.publish(chosen_path_);
      std::cout << std::fixed << std::setprecision(4) << "Corridor time cost : " << corridor_time << "ms. ";

      TicToc trajectory_generation_timer;
      trajectory_generation_timer.tic();
      generateTrajectory(&safe_corridor);
      auto trajectory_generation_time_duration = trajectory_generation_timer.toc();
      std::cout << std::fixed << std::setprecision(4) << "Trajectory generation time cost : " << trajectory_generation_time_duration << "ms. "
                << std::endl;
      historical_path_ = chosen_path_;
      historical_path_cost_ = chosen_path_cost_;
    }
  }

  exec_timer_.start();
  return;
}

void RPGPlanner::pruneGlobalPath(nav_msgs::Path &path_) {
  // iterate plan until a pose close the robot is found
  auto it = path_.poses.begin();
  auto erase_end = it;
  double weight_dist_ = 1.0, weight_yaw_ = 1.0;
  double dist_first_sqare = getPoseDiffCost(odom_pose_, path_.poses[0].pose, weight_dist_, weight_yaw_);
  while (it != path_.poses.end()) {
    double dist_sq = getPoseDiffCost(odom_pose_, it->pose, weight_dist_, weight_yaw_);
    if (dist_sq <= dist_first_sqare) {
      erase_end = it;
      dist_first_sqare = dist_sq;
    }
    ++it;
  }

  if (erase_end == path_.poses.end()) {
    path_.poses.resize(2);
    path_.poses[0].pose = odom_pose_;
    path_.poses[1].pose = goal_pose_;
  } else if (erase_end != path_.poses.begin()) {
    path_.poses.erase(path_.poses.begin(), erase_end);
  }
  return;
}

double RPGPlanner::getPoseDiffCost(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, double weight_dist, double weight_yaw) {
  double res = 0;
  double dx = a.position.x - b.position.x, dy = a.position.y - b.position.y;
  tf::Quaternion qa, qb;
  tf::quaternionMsgToTF(a.orientation, qa);
  tf::quaternionMsgToTF(b.orientation, qb);
  double dyaw = std::abs(qa.angleShortestPath(qb));
  res += weight_dist * (dx * dx + dy * dy) + weight_yaw * dyaw * dyaw;
  return res;
}

double RPGPlanner::calculatePathCost(nav_msgs::Path &path, double weight_dist, double weight_yaw) {
  double res = 0;
  if (path.poses.size() < 2) return 0;
  for (unsigned int i = 0; i < path.poses.size() - 1; ++i) {
    res += getPoseDiffCost(path.poses[i].pose, path.poses[i + 1].pose, weight_dist, weight_yaw);
  }
  return res;
}

bool RPGPlanner::isPoseEqual(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, double p_eps, double rad_eps) {
  tf::Quaternion qa, qb;
  tf::quaternionMsgToTF(a.orientation, qa);
  tf::quaternionMsgToTF(b.orientation, qb);
  if (std::abs(a.position.x - b.position.x) <= std::abs(p_eps) && std::abs(a.position.y - b.position.y) <= std::abs(p_eps) &&
      std::abs(qa.angleShortestPath(qb)) <= std::abs(rad_eps)) {
    return true;
  } else
    return false;
}

double RPGPlanner::getAbsoluteDistance(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b) {
  return std::abs(a.position.x - b.position.x) + std::abs(a.position.y - b.position.y);
}

void RPGPlanner::structCorridor(const nav_msgs::Path &path, std::vector<SafeRegion> *safe_corridor) {
  vec_E<Polyhedron2D> polys;
  int i = 1;
  while (i < path.poses.size() - 1) {
    EllipsoidDecomp2D decomp_util;
    decomp_util.set_obs(obs2d_);
    decomp_util.set_local_bbox(Vec2f(8, 6));
    vec_Vec2f cur_pts;
    cur_pts.push_back(Vec2f(path.poses[i - 1].pose.position.x, path.poses[i - 1].pose.position.y));
    cur_pts.push_back(Vec2f(path.poses[i].pose.position.x, path.poses[i].pose.position.y));
    decomp_util.dilate(cur_pts);
    polys.push_back(decomp_util.get_polyhedrons()[0]);
    for (auto &hyperplane : polys.back().vs_) {
      hyperplane.p_ -= hyperplane.n_ * DSTP_config_.safe_radius;
    }
    LinearConstraint2D linear_constraint(cur_pts[0], polys.back().hyperplanes());
    Vec2f pt(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    while (polys.back().inside(pt) && i < path.poses.size() - 1) {
      i++;
      pt = Vec2f(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    }
    safe_corridor->emplace_back(linear_constraint, i - 1);
  }
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
  poly_msg.header.frame_id = "map";
  cur_safe_corridor_pub_.publish(poly_msg);
}

bool RPGPlanner::generateTrajectory(std::vector<SafeRegion> *safe_corridor) {
  // DSTP optimizer
  int point_num = chosen_path_.poses.size();
  Eigen::VectorXd pts(chosen_path_.poses.size() * 3);

  for (size_t i = 0; i < chosen_path_.poses.size(); i++) {
    pts[i] = chosen_path_.poses[i].pose.position.x;
    pts[point_num + i] = chosen_path_.poses[i].pose.position.y;
    pts[point_num * 2 + i] = tf::getYaw(chosen_path_.poses[i].pose.orientation);
  }

  try {
    DSTP_path_ = chosen_path_;
    DSTP_opt_.optimizeTrajectory(init_state_, goal_state_, pts, DSTP_config_, safe_corridor, &DSTP_traj_, &DSTP_path_);
    DSTP_path_pub_.publish(DSTP_path_);
    DSTP_traj_pub_.publish(DSTP_traj_);
  } catch (std::runtime_error &e) {
    ROS_WARN(e.what());
    return false;
  }
  return true;
}

void RPGPlanner::odometryCallback(const nav_msgs::Odometry &msg) {
  odom_pose_ = msg.pose.pose;
  double yaw = tf::getYaw(odom_pose_.orientation);
  init_state_.head(5) << odom_pose_.position.x, odom_pose_.position.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z;

  flag_have_odom_ = true;
}

void RPGPlanner::goalCallback(geometry_msgs::PoseStamped::ConstPtr msg) {
  flag_have_goal_ = true;
  if (!isPoseEqual(goal_pose_, msg->pose, 0.1, 0.1)) {
    // new goal
    goal_pose_ = msg->pose;
    double yaw = tf::getYaw(msg->pose.orientation);
    goal_state_ << goal_pose_.position.x, goal_pose_.position.y, yaw, 0, 0;
  }
}

void RPGPlanner::gridmapCallback(const nav_msgs::OccupancyGrid &msg) {
  flag_have_gridmap_ = true;
  ROS_INFO("Receive a gridmap, the resolution is %.3f.", msg.info.resolution);
  origin_gridmap_ = msg;
  cell_radius_ = std::ceil(DSTP_config_.safe_radius / msg.info.resolution);
  map_edge_points.clear();
  InflateGridmap();
  updated_gridmap_ = origin_gridmap_;
  updated_gridmap_pub_.publish(updated_gridmap_);
  ROS_INFO("The cell radius = %d, updated_map width is %u and hight is %u.", cell_radius_, updated_gridmap_.info.width, updated_gridmap_.info.height);
}

void RPGPlanner::roughPathCallback(const nav_msgs::Path &msg) {
  new_rough_path_ = msg;

  ROS_DEBUG("Received the rough path, the point size is %lu.", msg.poses.size());
}

void RPGPlanner::accCallback(const std_msgs::Float64 &msg) { init_state_[5] = msg.data; }

int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};

bool RPGPlanner::isEdge(int x, int y) {
  if (origin_gridmap_.data[x * origin_gridmap_.info.width + y] > 90) {
    for (int i = 0; i < 4; ++i) {
      int a = x + dx[i], b = y + dy[i];
      if (a >= 0 && a < origin_gridmap_.info.height && b >= 0 && b < origin_gridmap_.info.width) {
        if (origin_gridmap_.data[a * origin_gridmap_.info.width + b] < 50) return true;
      }
    }
  }
  return false;
}

void RPGPlanner::inflateGridPoint(nav_msgs::OccupancyGrid &map, int h, int w, int cell_radius) {
  int h_min, h_max, w_min, w_max;
  h_min = std::max(h - cell_radius, 0), h_max = std::min(h + cell_radius, static_cast<int>(origin_gridmap_.info.height) - 1);
  for (int a = h_min; a <= h_max; ++a) {
    int dw = cell_radius;
    w_min = std::max(w - dw, 0), w_max = std::min(w + dw, static_cast<int>(origin_gridmap_.info.width) - 1);
    for (int b = w_min; b <= w_max; ++b) {
      if (map.data[a * map.info.width + b] < 50) {
        map.data[a * map.info.width + b] = GRIDMAP_TYPE::OBS_INFLATION;
      }
    }
  }
}

void RPGPlanner::InflateGridmap() {
  if (cell_radius_ == 0) return;
  const double resolution = origin_gridmap_.info.resolution;
  for (int i = 0; i < origin_gridmap_.info.height; ++i) {
    for (int j = 0; j < origin_gridmap_.info.width; ++j) {
      if (isEdge(i, j)) {
        map_edge_points.push_back({i, j});
        origin_gridmap_.data[i * origin_gridmap_.info.width + j] = GRIDMAP_TYPE::OBS_EDGE;
        obs2d_.emplace_back(j * resolution, i * resolution);
      }
    }
  }
  obs_cloud_static_.points.clear();
  unsigned int edge_pt_num = (origin_gridmap_.info.height + origin_gridmap_.info.width) * 2;
  obs_cloud_static_.points.reserve(map_edge_points.size() * 5 + edge_pt_num);
  obs_cloud_static_.channels.resize(1);
  obs_cloud_static_.channels[0].values.reserve(obs_cloud_static_.points.size());
  geometry_msgs::Point32 point;
  for (auto &pt : map_edge_points) {
    inflateGridPoint(origin_gridmap_, pt.first, pt.second, cell_radius_);
  }

  geometry_msgs::Point32 point_;
  for (unsigned int w = 0; w < origin_gridmap_.info.width; ++w) {
    point_.x = w * resolution;
    point_.y = 0;
    obs_cloud_static_.points.push_back(point_);
    obs_cloud_static_.channels[0].values.push_back(255);
    point_.y = (origin_gridmap_.info.height - 1) * resolution;
    obs_cloud_static_.points.push_back(point_);
    obs_cloud_static_.channels[0].values.push_back(255);
    obs2d_.emplace_back(w * resolution, 0.0);
    obs2d_.emplace_back(w * resolution, (origin_gridmap_.info.height - 1) * resolution);
  }
  for (unsigned int h = 0; h < origin_gridmap_.info.height; ++h) {
    point_.x = 0;
    point_.y = h * resolution;
    obs_cloud_static_.points.push_back(point_);
    obs_cloud_static_.channels[0].values.push_back(255);
    point_.x = (origin_gridmap_.info.width - 1) * resolution;
    obs_cloud_static_.points.push_back(point_);
    obs_cloud_static_.channels[0].values.push_back(255);
    obs2d_.emplace_back(0.0, h * resolution);
    obs2d_.emplace_back((origin_gridmap_.info.width - 1) * resolution, h * resolution);
  }
  obs_cloud_total_.points = obs_cloud_static_.points;
  obs_cloud_total_.channels = obs_cloud_static_.channels;
  obs_cloud_total_.header.stamp = ros::Time::now();
  obstacle_pointcloud_pub_.publish(obs_cloud_total_);
}

} // namespace dstp_planner