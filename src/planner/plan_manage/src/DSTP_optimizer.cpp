#include <ros/ros.h>

#include "optimizer/DSTP_optimizer.h"

namespace dstp_planner {

double DSTPProblem::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
  double fx = 0.0;
  grad = Eigen::VectorXd::Zero(x.size());

  // start point
  fx += w_pos * (pow(x[0] - init_state_[0], 2) + pow(x[y_offset_] - init_state_[1], 2) + pow(cos(x[yaw_offset_]) - cos(init_state_[2]), 2) +
                 pow(sin(x[yaw_offset_]) - sin(init_state_[2]), 2) + pow(x[v_offset_] - init_state_[3], 2) +
                 pow(x[kappa_offset_] - init_state_[4], 2) + pow(x[a_offset_] - init_state_[5], 2));
  grad[0] += w_pos * 2 * (x[0] - init_state_[0]);
  grad[y_offset_] += w_pos * 2 * (x[y_offset_] - init_state_[1]);
  grad[yaw_offset_] += w_pos * 2 * (cos(x[yaw_offset_]) - cos(init_state_[2])) * -sin(x[yaw_offset_]);
  grad[yaw_offset_] += w_pos * 2 * (sin(x[yaw_offset_]) - sin(init_state_[2])) * cos(x[yaw_offset_]);
  grad[v_offset_] += w_pos * 2 * (x[v_offset_] - init_state_[3]);
  grad[kappa_offset_] += w_pos * 2 * (x[kappa_offset_] - init_state_[4]);
  grad[a_offset_] += w_pos * 2 * (x[a_offset_] - init_state_[5]);
  // end point
  fx += w_pos * (pow(x[point_num_ - 1] - fin_state_[0], 2) + pow(x[y_offset_ + point_num_ - 1] - fin_state_[1], 2) +
                 pow(cos(x[yaw_offset_ + point_num_ - 1]) - cos(fin_state_[2]), 2) +
                 pow(sin(x[yaw_offset_ + point_num_ - 1]) - sin(fin_state_[2]), 2) + pow(x[v_offset_ + point_num_ - 1] - fin_state_[3], 2) +
                 pow(x[kappa_offset_ + point_num_ - 1] - fin_state_[4], 2) + pow(x[a_offset_ + point_num_ - 1], 2));
  grad[point_num_ - 1] += w_pos * 2 * (x[point_num_ - 1] - fin_state_[0]);
  grad[y_offset_ + point_num_ - 1] += w_pos * 2 * (x[y_offset_ + point_num_ - 1] - fin_state_[1]);
  grad[yaw_offset_ + point_num_ - 1] +=
      w_pos * 2 * (cos(x[yaw_offset_ + point_num_ - 1]) - cos(fin_state_[2])) * -sin(x[yaw_offset_ + point_num_ - 1]);
  grad[yaw_offset_ + point_num_ - 1] +=
      w_pos * 2 * (sin(x[yaw_offset_ + point_num_ - 1]) - sin(fin_state_[2])) * cos(x[yaw_offset_ + point_num_ - 1]);
  grad[v_offset_ + point_num_ - 1] += w_pos * 2 * (x[v_offset_ + point_num_ - 1] - fin_state_[3]);
  grad[kappa_offset_ + point_num_ - 1] += w_pos * 2 * (x[kappa_offset_ + point_num_ - 1] - fin_state_[4]);
  grad[a_offset_ + point_num_ - 1] = w_pos * 2 * x[a_offset_ + point_num_ - 1];

  double grad_func_val, tmp;
  for (int i = 0; i < point_num_ - 2; i++) {
    fx += 0.5 * pow(x[t_offset_ + i + 1] - x[t_offset_ + i], 2);
    grad[t_offset_ + i] += 0.5 * -2 * (x[t_offset_ + i + 1] - x[t_offset_ + i]);
    grad[t_offset_ + i + 1] += 0.5 * 2 * (x[t_offset_ + i + 1] - x[t_offset_ + i]);
  }
  for (int i = 0; i < point_num_ - 1; i++) {
    // cost function
    // for T
    fx += w_t * x[t_offset_ + i];
    grad[t_offset_ + i] += w_t;

    // for kappa^2
    fx += w_kappa * pow(x[kappa_offset_ + i], 2);
    grad[kappa_offset_ + i] += w_kappa * 2 * x[kappa_offset_ + i];

    fx += w_jerk * pow(x[a_offset_ + i + 1] - x[a_offset_ + i], 2) / x[t_offset_ + i];
    grad[a_offset_ + i + 1] += w_jerk * 2 * (x[a_offset_ + i + 1] - x[a_offset_ + i]) / x[t_offset_ + i];
    grad[a_offset_ + i] += w_jerk * -2 * (x[a_offset_ + i + 1] - x[a_offset_ + i]) / x[t_offset_ + i];
    grad[t_offset_ + i] += w_jerk * pow(x[a_offset_ + i + 1] - x[a_offset_ + i], 2) / -pow(x[t_offset_ + i], 2);

    // x[i+1] = x[i]+0.5(v[i]*cos(theta[i])+v[i+1]*cos(theta[i+1]))*t[i]
    double cos_beta = cos(x[yaw_offset_ + i]);
    double sin_beta = sin(x[yaw_offset_ + i]);
    double cos_beta1 = cos(x[yaw_offset_ + i + 1]);
    double sin_beta1 = sin(x[yaw_offset_ + i + 1]);
    double F = x[i + 1] - x[i] - 0.5 * (x[v_offset_ + i] * cos_beta + x[v_offset_ + i + 1] * cos_beta1) * x[t_offset_ + i];
    fx += w_F * F * F;
    grad_func_val = w_F * 2 * F;
    grad[i] += -grad_func_val;
    grad[i + 1] += grad_func_val;
    grad[v_offset_ + i] += grad_func_val * -0.5 * cos_beta * x[t_offset_ + i];
    grad[v_offset_ + i + 1] += grad_func_val * -0.5 * cos_beta1 * x[t_offset_ + i];
    grad[yaw_offset_ + i] += grad_func_val * -0.5 * x[v_offset_ + i] * -sin_beta * x[t_offset_ + i];
    grad[yaw_offset_ + i + 1] += grad_func_val * -0.5 * x[v_offset_ + i + 1] * -sin_beta1 * x[t_offset_ + i];
    grad[t_offset_ + i] += grad_func_val * -0.5 * (x[v_offset_ + i] * cos_beta + x[v_offset_ + i + 1] * cos_beta1);

    // y[i+1] = y[i]+0.5(v[i]*sin(theta[i])+v[i+1]*sin(theta[i+1]))*t[i]
    F = x[y_offset_ + i + 1] - x[y_offset_ + i] - 0.5 * (x[v_offset_ + i] * sin_beta + x[v_offset_ + i + 1] * sin_beta1) * x[t_offset_ + i];
    fx += w_F * F * F;
    grad_func_val = w_F * 2 * F;
    grad[y_offset_ + i] += -grad_func_val;
    grad[y_offset_ + i + 1] += grad_func_val;
    grad[v_offset_ + i] += grad_func_val * -0.5 * sin_beta * x[t_offset_ + i];
    grad[v_offset_ + i + 1] += grad_func_val * -0.5 * sin_beta1 * x[t_offset_ + i];
    grad[yaw_offset_ + i] += grad_func_val * -0.5 * x[v_offset_ + i] * cos_beta * x[t_offset_ + i];
    grad[yaw_offset_ + i + 1] += grad_func_val * -0.5 * x[v_offset_ + i + 1] * cos_beta1 * x[t_offset_ + i];
    grad[t_offset_ + i] += grad_func_val * -0.5 * (x[v_offset_ + i] * sin_beta + x[v_offset_ + i + 1] * sin_beta1);

    // v[i+1] = v[i]+a[i]*t[i]
    double V = x[v_offset_ + i + 1] - (x[v_offset_ + i] + x[a_offset_ + i] * x[t_offset_ + i]);
    fx += w_V * V * V;
    grad_func_val = w_V * 2 * V;
    grad[v_offset_ + i] += -grad_func_val;
    grad[v_offset_ + i + 1] += grad_func_val;
    grad[a_offset_ + i] += grad_func_val * -x[t_offset_ + i];
    grad[t_offset_ + i] += grad_func_val * -x[a_offset_ + i];

    // for orientation
    double cos_b = cos(x[yaw_offset_ + i] +
                       0.5 * (x[kappa_offset_ + i] * x[v_offset_ + i] + x[kappa_offset_ + i + 1] * x[v_offset_ + i + 1]) * x[t_offset_ + i]);
    double sin_b = sin(x[yaw_offset_ + i] +
                       0.5 * (x[kappa_offset_ + i] * x[v_offset_ + i] + x[kappa_offset_ + i + 1] * x[v_offset_ + i + 1]) * x[t_offset_ + i]);
    double cos_b1 = cos(x[yaw_offset_ + i + 1]);
    double sin_b1 = sin(x[yaw_offset_ + i + 1]);
    fx += w_W * (pow(cos_b1 - cos_b, 2) + pow(sin_b1 - sin_b, 2));
    grad_func_val = w_W * 2 * (cos_b1 - cos_b);
    grad[yaw_offset_ + i + 1] += grad_func_val * -sin_b1;
    grad[yaw_offset_ + i] += grad_func_val * sin_b;
    grad[kappa_offset_ + i] += grad_func_val * sin_b * 0.5 * x[v_offset_ + i] * x[t_offset_ + i];
    grad[kappa_offset_ + i + 1] += grad_func_val * sin_b * 0.5 * x[v_offset_ + i + 1] * x[t_offset_ + i];
    grad[v_offset_ + i] += grad_func_val * sin_b * 0.5 * x[kappa_offset_ + i] * x[t_offset_ + i];
    grad[v_offset_ + i + 1] += grad_func_val * sin_b * 0.5 * x[kappa_offset_ + i + 1] * x[t_offset_ + i];
    grad[t_offset_ + i] += grad_func_val * sin_b * 0.5 * (x[kappa_offset_ + i] * x[v_offset_ + i] + x[kappa_offset_ + i + 1] * x[v_offset_ + i + 1]);

    grad_func_val = w_W * 2 * (sin_b1 - sin_b);
    grad[yaw_offset_ + i + 1] += grad_func_val * cos_b1;
    grad[yaw_offset_ + i] += grad_func_val * -cos_b;
    grad[kappa_offset_ + i] += grad_func_val * -cos_b * 0.5 * x[v_offset_ + i] * x[t_offset_ + i];
    grad[kappa_offset_ + i + 1] += grad_func_val * -cos_b * 0.5 * x[v_offset_ + i + 1] * x[t_offset_ + i];
    grad[v_offset_ + i] += grad_func_val * -cos_b * 0.5 * x[kappa_offset_ + i] * x[t_offset_ + i];
    grad[v_offset_ + i + 1] += grad_func_val * -cos_b * 0.5 * x[kappa_offset_ + i + 1] * x[t_offset_ + i];
    grad[t_offset_ + i] += grad_func_val * -cos_b * 0.5 * (x[kappa_offset_ + i] * x[v_offset_ + i] + x[kappa_offset_ + i + 1] * x[v_offset_ + i + 1]);

    // gear shifting position
    tmp = -x[v_offset_ + i] * x[v_offset_ + i + 1];
    if (tmp > 0.0) {
      fx += w_GS * pow(tmp, 3);
      grad[v_offset_ + i] += w_GS * 3 * pow(tmp, 2) * -x[v_offset_ + i + 1];
      grad[v_offset_ + i + 1] += w_GS * 3 * pow(tmp, 2) * -x[v_offset_ + i];
    }
  }
  // for obstacle
  int pose_i = 1;
  double Axb;
  for (const auto &safe_region : safe_corridor_) {
    const auto &A = safe_region.linear_constraint.A();
    const auto &b = safe_region.linear_constraint.b();
    while (pose_i <= safe_region.poly_end) {
      double cos_beta = cos(x[yaw_offset_ + pose_i]);
      double sin_beta = sin(x[yaw_offset_ + pose_i]);
      for (int edge = 0; edge < A.rows(); edge++) {
        // front point
        Axb = A(edge, 0) * (x[pose_i] + DSTP_config_.wheel_base * cos_beta) +
              A(edge, 1) * (x[y_offset_ + pose_i] + DSTP_config_.wheel_base * sin_beta) - b[edge];
        if (Axb > 0.0) {
          fx += w_obs * pow(Axb, 3);
          grad_func_val = w_obs * 3 * pow(Axb, 2);
          grad[pose_i] += grad_func_val * A(edge, 0);
          grad[y_offset_ + pose_i] += grad_func_val * A(edge, 1);
          grad[yaw_offset_ + pose_i] +=
              grad_func_val * (A(edge, 0) * DSTP_config_.wheel_base * -sin_beta + A(edge, 1) * DSTP_config_.wheel_base * cos_beta);
        }
        // rear point
        Axb = A(edge, 0) * x[pose_i] + A(edge, 1) * x[y_offset_ + pose_i] - b(edge);
        if (Axb > 0) {
          fx += w_obs * pow(Axb, 3);
          grad_func_val = w_obs * 3 * pow(Axb, 2);
          grad[pose_i] += grad_func_val * A(edge, 0);
          grad[y_offset_ + pose_i] += grad_func_val * A(edge, 1);
        }
      }
      ++pose_i;
    }
  }
  // We have to constrint the last point of safe_region[i] within the safe_region[i+1]
  for (size_t i = 1; i < safe_corridor_.size(); ++i) {
    const auto &A = safe_corridor_[i].linear_constraint.A();
    const auto &b = safe_corridor_[i].linear_constraint.b();
    pose_i = safe_corridor_[i - 1].poly_end;
    double cos_beta = cos(x[yaw_offset_ + pose_i]);
    double sin_beta = sin(x[yaw_offset_ + pose_i]);
    for (int edge = 0; edge < A.rows(); edge++) {
      // front point
      Axb = A(edge, 0) * (x[pose_i] + DSTP_config_.wheel_base * cos_beta) +
            A(edge, 1) * (x[y_offset_ + pose_i] + DSTP_config_.wheel_base * sin_beta) - b(edge);
      if (Axb > 0.0) {
        fx += w_obs * pow(Axb, 3);
        grad_func_val = w_obs * 3 * pow(Axb, 2);
        grad[pose_i] += grad_func_val * A(edge, 0);
        grad[y_offset_ + pose_i] += grad_func_val * A(edge, 1);
        grad[yaw_offset_ + pose_i] +=
            grad_func_val * (A(edge, 0) * DSTP_config_.wheel_base * -sin_beta + A(edge, 1) * DSTP_config_.wheel_base * cos_beta);
      }
      // end point
      Axb = A(edge, 0) * x[pose_i] + A(edge, 1) * x[y_offset_ + pose_i] - b(edge);
      if (Axb > 0.0) {
        fx += w_obs * pow(Axb, 3);
        grad_func_val = w_obs * 3 * pow(Axb, 2);
        grad[pose_i] += grad_func_val * A(edge, 0);
        grad[y_offset_ + pose_i] += grad_func_val * A(edge, 1);
      }
    }
  }
  return fx;
} // namespace rpg_planner

bool DSTPOptimizer::optimizeTrajectory(const Eigen::VectorXd &iniState, const Eigen::VectorXd &finState, const Eigen::VectorXd &pts,
                                       const DSTPParameter &DSTP_config, std::vector<SafeRegion> *safe_corridor, dstp_planner::DSTPtraj *DSTP_traj,
                                       nav_msgs::Path *DSTP_path) {
  const int point_num = pts.rows() / 3;
  const int y_offset = point_num;
  const int yaw_offset = point_num * 2;
  const int v_offset = point_num * 3;
  const int a_offset = point_num * 4;
  const int kappa_offset = point_num * 5;
  const int t_offset = point_num * 6;

  // Set up parameters
  LBFGSpp::LBFGSBParam<double> param; // New parameter class
  param.epsilon = 1e-4;
  param.epsilon_rel = 1e-4;
  // param.delta = 1e-5;
  param.max_iterations = 700;
  param.max_linesearch = 100;

  // Create solver and function object
  LBFGSpp::LBFGSBSolver<double> solver(param); // New solver class
  DSTPProblem problem(point_num, iniState, finState, DSTP_config, *safe_corridor);

  // set the box constraint
  Eigen::VectorXd ub(7 * point_num - 1);
  ub.head(point_num * 3) = Eigen::VectorXd::Constant(point_num * 3, 1e4);                            // x ,y ,beta
  ub.segment(v_offset, point_num) = Eigen::VectorXd::Constant(point_num, DSTP_config.max_vel);       // v
  ub.segment(a_offset, point_num) = Eigen::VectorXd::Constant(point_num, DSTP_config.max_acc);       // a
  ub.segment(kappa_offset, point_num) = Eigen::VectorXd::Constant(point_num, DSTP_config.max_kappa); // kappa
  ub.segment(t_offset, point_num - 1) = Eigen::VectorXd::Constant(point_num - 1, 10.0);              // T
  Eigen::VectorXd lb = -ub;
  lb.segment(v_offset, point_num) = Eigen::VectorXd::Constant(point_num, DSTP_config.min_vel); // v
  lb.segment(t_offset, point_num - 1) = Eigen::VectorXd::Constant(point_num - 1, 0.1);         // T

  // initial guess
  Eigen::VectorXd x(7 * point_num - 1);
  if (last_point_ == 0) {
    x.head(point_num * 3) = pts;
    x.segment(point_num * 3, point_num * 3) = Eigen::VectorXd::Constant(point_num * 3, 0);
    x.segment(point_num * 6, point_num - 1) = Eigen::VectorXd::Constant(point_num - 1, 0.25);
  } else if (last_point_ == point_num) {
    x = last_x_;
  } else if (last_point_ < point_num) {
    x.head(point_num * 3) = pts;
    x.segment(v_offset, last_point_) = last_x_.segment(last_point_ * 3, last_point_); // v
    x.segment(point_num * 3 + last_point_, point_num - last_point_) = Eigen::VectorXd::Constant(point_num - last_point_, 0);
    x.segment(point_num * 4, last_point_) = last_x_.segment(last_point_ * 4, last_point_); // a
    x.segment(point_num * 4 + last_point_, point_num - last_point_) = Eigen::VectorXd::Constant(point_num - last_point_, 0);
    x.segment(point_num * 5, last_point_) = last_x_.segment(last_point_ * 5, last_point_); // kappa
    x.segment(point_num * 5 + last_point_, point_num - last_point_) = Eigen::VectorXd::Constant(point_num - last_point_, 0);
    x.segment(point_num * 6, last_point_ - 1) = last_x_.segment(last_point_ * 6, last_point_ - 1); // T
    x.segment(point_num * 6 + last_point_ - 1, point_num - last_point_) = Eigen::VectorXd::Constant(point_num - last_point_, 0.25);
  } else {
    int diff = last_point_ - point_num;
    for (int i = 0; i < 6; i++) {
      x.segment(point_num * i, point_num) = last_x_.segment(last_point_ * i + diff, point_num);
    }
    x.segment(point_num * 6, point_num - 1) = last_x_.segment(last_point_ * 6 + diff, point_num - 1); // T
  }
  x[0] = iniState[0];
  x[y_offset] = iniState[1];
  x[yaw_offset] = iniState[2];
  x[v_offset] = iniState[3];
  x[a_offset] = iniState[5];
  x[kappa_offset] = iniState[4];

  double fx;
  solver.minimize(problem, x, fx, lb, ub);
  ROS_WARN("fx = %lf", fx);
  last_point_ = point_num;
  last_x_ = x;

  for (size_t i = 0; i < DSTP_path->poses.size(); i++) {
    DSTP_path->poses[i].pose.position.x = x[i];
    DSTP_path->poses[i].pose.position.y = x[y_offset + i];
    DSTP_path->poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(x[yaw_offset + i]);
  }
  DSTP_traj->v.data.resize(point_num);
  DSTP_traj->a.data.resize(point_num);
  DSTP_traj->kappa.data.resize(point_num);
  DSTP_traj->t.data.resize(point_num - 1);
  for (int i = 0; i < point_num; i++) {
    DSTP_traj->v.data[i] = x[v_offset + i];
    DSTP_traj->a.data[i] = x[a_offset + i];
    DSTP_traj->kappa.data[i] = x[kappa_offset + i];
  }
  for (int i = 0; i < point_num - 1; i++) {
    DSTP_traj->t.data[i] = x[t_offset + i];
  }

  return true;
}
} // namespace dstp_planner