# DSTP-planner

## Prerequisite
- ROS
- [DecompRos](https://github.com/sikang/DecompROS/tree/master) (Please follow the DecompRos repository)


## Acknowledgements

1.  The safe corridor construction is from [DecompRos](https://github.com/sikang/DecompROS/tree/master)
2.  The L-BFGS-B solver is from [LBFGSpp](https://github.com/yixuan/LBFGSpp). It is a header-only C++ library that implements the Limited-memory BFGS algorithm (L-BFGS) for unconstrained minimization problems, and a modified version of the L-BFGS-B algorithm for box-constrained ones.
3.  The simulation environment is based on [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

## Compilation

```bash
mkdir DSTP_ws && cd DSTP_ws
git clone https://github.com/ziyi-joe/DSTP
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch dstp_planner car_map.launch
roslaunch dstp_planner car_motion.launch
