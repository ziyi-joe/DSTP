#ifndef __DSTP_OPTIMIZER_H__
#define __DSTP_OPTIMIZER_H__

#include <nav_msgs/Path.h>
#include <Eigen/Eigen>

#include "optimizer/LBFGSB.h"
#include "dstp_planner/DSTPtraj.h"
#include "tf/transform_broadcaster.h"
#include "decomp_geometry/polyhedron.h"

namespace dstp_planner {
struct SafeRegion {
  LinearConstraint2D linear_constraint;
  int poly_end; //表示每个poly中的最后一个点的下标
  SafeRegion(LinearConstraint2D l, int p) : linear_constraint(l), poly_end(p) {}
};

struct DSTPParameter {
  double min_vel = -1.0;
  double max_vel = 1.0;
  double max_acc = 1.0;
  double max_kappa = 1.0;
  double safe_radius = 0.1;
  double wheel_base = 0.1;
};

class DSTPProblem {
public:
  DSTPProblem() = default;
  DSTPProblem(int n, Eigen::VectorXd init_state, Eigen::VectorXd fin_state, DSTPParameter DSTP_config, std::vector<SafeRegion> &safe_corridor)
      : point_num_(n), init_state_(init_state), fin_state_(fin_state) {
    y_offset_ = n;
    yaw_offset_ = 2 * n;
    v_offset_ = 3 * n;
    a_offset_ = 4 * n;
    kappa_offset_ = 5 * n;
    t_offset_ = 6 * n;
    safe_corridor_ = std::move(safe_corridor);
    DSTP_config_ = std::move(DSTP_config);
  }
  double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad);

private:
  int point_num_;
  int y_offset_, yaw_offset_, v_offset_, a_offset_, kappa_offset_, t_offset_;
  Eigen::VectorXd init_state_, fin_state_;
  double w_t = 0.25, w_kappa = 1.0, w_jerk = 1.0, w_pos = 100, w_F = 1000, w_V = 100, w_W = 1000, w_GS = 100, w_obs = 100, a0 = 1e-3;
  DSTPParameter DSTP_config_;
  std::vector<SafeRegion> safe_corridor_;
};

class DSTPOptimizer {
public:
  DSTPOptimizer() = default;
  /**
   * @brief main planning API
   *
   * @param iniState 初始状态，为x,y,yaw,v,w
   * @param finState
   * @param pts
   * @param max_v
   * @param max_along
   * @param min_R
   * @param chosen_corridor_
   * @param DSTP_traj 优化后的轨迹消息,v,a,w,t
   * @return true
   * @return false
   */
  bool optimizeTrajectory(const Eigen::VectorXd &iniState, const Eigen::VectorXd &finState, const Eigen::VectorXd &pts,
                          const DSTPParameter &DSTP_config, std::vector<SafeRegion> *safe_corridor, dstp_planner::DSTPtraj *DSTP_traj,
                          nav_msgs::Path *DSTP_path);

private:
  int last_point_ = 0;
  Eigen::VectorXd last_x_; // last solution, for the next initial guess
};
} // namespace dstp_planner
#endif