#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/rpg_planner.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rpg_planner_node");
  ros::NodeHandle nh("~");

  dstp_planner::RPGPlanner rpg_planner(nh);

  rpg_planner.init();

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
