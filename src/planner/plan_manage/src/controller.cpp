#include <ros/ros.h>
#include "dstp_planner/DSTPtraj.h"
#include "plan_manage/tic_toc.h"

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

double v = 0, kappa = 0, t = 0, a = 0, seg = 0;
dstp_planner::DSTPtrajPtr traj;
geometry_msgs::Pose goal;
enum State { kWaiting = 0, kReciving, kRunning };
enum State state = kWaiting;
TicToc timer;

void TrajCallback(dstp_planner::DSTPtrajPtr msg) {
  traj = msg;
  t = 0;
  seg = 0;
  // w = msg->w.data[0];
  if (state == kReciving) {
    timer.tic();
    state = kRunning;
  }
}
void OdometryCallback(const nav_msgs::OdometryConstPtr msg) {
  // v = msg->twist.twist.linear.x;
  // w = msg->twist.twist.angular.z;
  if (std::abs(msg->pose.pose.position.x - goal.position.x) <= 0.1 && std::abs(msg->pose.pose.position.y - goal.position.y) <= 0.1 &&
      std::abs(v) <= 0.02) {
    auto duration = timer.toc();
    std::cout << std::fixed << std::setprecision(4) << "Running time cost : " << duration << "ms. " << std::endl;
    state = kWaiting;
  }
}

void GoalCallback(geometry_msgs::PoseStamped::ConstPtr msg) {
  state = kReciving;
  goal = msg->pose;
}

int main(int argc, char** args) {
  ros::init(argc, args, "controller node");
  ros::NodeHandle nh("~");

  ros::Subscriber DSTP_sub = nh.subscribe("/DSTP_traj", 1, TrajCallback);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, OdometryCallback);
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher a_pub = nh.advertise<std_msgs::Float64>("/now_a", 1);

  double rate = 10;
  ros::Rate loop_rate = rate;
  geometry_msgs::Twist cmd_vel;
  std_msgs::Float64 now_a;

  while (ros::ok()) {
    ros::spinOnce();
    if (state != kRunning) {
      v = kappa = 0;
      cmd_vel.linear.x = v;
      cmd_vel.angular.z = kappa;
      cmd_pub.publish(cmd_vel);
      loop_rate.sleep();
      continue;
    }
    t += 1 / rate;
    if (seg < traj->t.data.size() && t > traj->t.data[seg]) {
      auto duration = timer.toc();
      std::cout << std::fixed << std::setprecision(4) << "Running time cost : " << duration << "ms. " << std::endl;
      ROS_WARN("t is bigger than seg0 %lf", t);
      t -= traj->t.data[seg];
      seg++;
      // w = traj->w.data[seg];
    }
    if (seg >= traj->t.data.size()) {
      v = kappa = 0;
    } else {
      a += (traj->a.data[seg + 1] - traj->a.data[seg]) / traj->t.data[seg] / rate;
      v += a / rate;
      kappa += (traj->kappa.data[seg + 1] - traj->kappa.data[seg]) / traj->t.data[seg] / rate;
      now_a.data = a;
      // w = traj->w.data[seg];
    }
    a_pub.publish(now_a);
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = kappa * v;
    cmd_pub.publish(cmd_vel);
    if (v >= 3.01) {
      ROS_WARN("pub v=%lf,w=%lf,a=%lf", v, kappa * v, a);
    }
    // ROS_INFO("pub v=%lf,w=%lf,a=%lf", v, w, a);
    loop_rate.sleep();
  }
  ros::shutdown();
}