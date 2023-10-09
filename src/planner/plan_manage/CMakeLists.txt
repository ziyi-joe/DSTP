cmake_minimum_required(VERSION 2.8.3)
project(dstp_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
# ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  path_searching
  nav_msgs
  message_generation
  decomp_ros_utils
  decomp_ros_msgs
  visualization_msgs
)

add_message_files(
  FILES DSTPtraj.msg
)

generate_messages( 
  DEPENDENCIES std_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rpg_planner
 CATKIN_DEPENDS path_searching
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(DSTP_optimizer
  src/DSTP_optimizer.cpp
)
target_link_libraries(DSTP_optimizer 
  ${catkin_LIBRARIES}
)

add_executable(rpg_planner_node
  src/rpg_planner_node.cpp 
  src/rpg_planner.cpp
)
target_link_libraries(rpg_planner_node 
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  DSTP_optimizer
)

add_executable(controller_node
  src/controller.cpp
)

target_link_libraries(controller_node 
  ${catkin_LIBRARIES}
)