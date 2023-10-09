#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include "path_searching/tic_toc.h"
#include "path_searching/hybrid_a.h"

nav_msgs::OccupancyGridPtr grid_map = nullptr;
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal;
bool have_odom = false;
bool have_goal = false;
bool have_map = false;

void MapCallback(nav_msgs::OccupancyGridPtr msg) {
  grid_map = msg;
  have_map = true;
}

void OdomCallback(nav_msgs::OdometryPtr msg) {
  start_pose = msg->pose.pose;
  have_odom = true;
}

void GoalCallback(geometry_msgs::PoseStamped::ConstPtr msg) {
  goal = msg->pose;
  ROS_INFO("Got new goal:%.3f,%.3f", msg->pose.position.x, msg->pose.position.y);
  have_goal = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hybrid_A");
  ros::NodeHandle nh("~");
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/rough_hybrid_a", 10);
  ros::Subscriber map_sub = nh.subscribe("/updated_gridmap", 1, MapCallback);
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, OdomCallback);

  double minimal_turning_radius;

  nh.getParam("MinimalTurningRadius", minimal_turning_radius);
  ros::Rate loop_rate = 10;

  TicToc hybrid_a_timer;
  while (ros::ok()) {
    ros::spinOnce();

    if (have_goal && have_map && have_odom) {
      hybrid_a_timer.tic();
      nav_msgs::Path plan = HybridASearch(start_pose, goal, minimal_turning_radius, grid_map);
      path_pub.publish(plan);
      const auto hybrid_a_search_duration = hybrid_a_timer.toc();
      std::cout << std::fixed << std::setprecision(4) << "hybrid_a_star search duration :" << hybrid_a_search_duration << " ms" << std::endl;

      have_goal = false;
    }

    loop_rate.sleep();
  }

  ros::shutdown();
}