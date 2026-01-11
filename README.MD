## Dual-arm movement setup
This repository contains a runnable simulation where two Franka FR3 arms work in tandem in the same work area.
The simulation is done with Gazebo Harmonic, and the package utilizes franka_ros2, MoveIt, MoveItPy and ROS2 Jazzy.

## Setup

## Prerequisites
This simulation has been developed with franka_ros2 Jazzy branch which is at the time of writing still in development.
The simulation's functionality is assured up to the following commit, and tag:
commit cc6d691eb3160811e553bde6f9820987b2ffe6f2, tag: v3.1.1, origin/jazzy
Functionality with newer commits can not be assumed.

The functionality of the simulation has been tested with a WSL Ubuntu-24.04 Image as well as on native Ubuntu-24.04 installation.



## Running the simulation
```bash
source /opt/ros/jazzy/setup.bash
```
```bash
source ~/franka_ros2_ws/install/setup.bash
```
```bash
source install/setup.bash
```
To manually add the robot arm resource information to gazebo run
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/franka_ros2_ws/install/franka_description/share
```

The simulation can be launched with the following command
```bash
ros2 launch fr3_dual_arm_sim simulation.launch.py
```

## Contact
For questions or support regarding the package, please open an issue on the [GitHub Issues](https://github.com/AtteNyyssonen/robotics_project_work/issues) page.
