cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  nav_msgs
  tf
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${traj_opt_INCLUDE_DIRS}
)

add_library( path_searching 
    src/node_hybrid.cpp
    src/hybrid_a.cpp
    )
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
    ${OMPL_LIBRARIES}
)

add_library( hybrid_A 
    src/node_hybrid.cpp
    src/hybrid_a.cpp
    )
target_link_libraries( hybrid_A
    ${catkin_LIBRARIES} 
    ${OMPL_LIBRARIES}
)

add_executable(global_path_generator
    src/global_path_generator.cpp
)

target_link_libraries(global_path_generator
    ${catkin_LIBRARIES}
    hybrid_A
)
