## Dual-arm movement setup
This repository contains a runnable simulation where two Franka FR3 arms work in tandem in the same work area.
The simulation is done with Gazebo Harmonic, and the package utilizes franka_ros2, MoveIt, MoveItPy and ROS2 Jazzy.

## Setup

## Prerequisites
1. **Install ROS 2 Jazzy Development environment**
Follow this official instruction guide to install **ROS 2 Jazzy "Desktop Install"** clicking the link below
[**here**](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Install the **Development Tools** package:
```bash
sudo apt install ros-dev-tools
```

Source the installed **ROS 2 Jazzy** environment:
```bash
source /opt/ros/jazzy/setup.bash
```

2. **Install the ROS 2 Integration for Franka Robotics Research Robots**
Follow the README.md instruction guide to install **FRANKA_ROS2** clicking the link below
[**here**](https://github.com/frankarobotics/franka_ros2/blob/jazzy/README.md)

Skip to part 2, because **ROS 2 Jazzy Development** environment was installed in the last step of this tutorial.

This simulation has been developed with franka_ros2 Jazzy branch which is at the time of writing still in development.
The simulation's functionality is assured up to the following commit, and tag:
commit cc6d691eb3160811e553bde6f9820987b2ffe6f2, tag: v3.1.1, origin/jazzy
Functionality with newer commits can not be assured.

Once installation is completed remember to source the built workspace:
```bash
source install/setup.bash
```

3. **Install the dependencies of the dual arm robot setup package**
**Create an initial folder for the ROS 2 Workspace:**
   ```bash
   mkdir ~/dual_arm_robot
   cd ~/dual_arm_robot
   ```
**Clone the Repository:**
   ```bash
    git clone https://github.com/AtteNyyssonen/robotics_project_work.git
    cd ~/dual_arm_robot/dual_arm_robot_setup
   ```
**Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
   ```
**Build**
   ```bash
   # Make sure you have sourced both the ROS2 Jazzy installation and the franka_ros2 installation before building
   colcon build
   ```
**Adjust Enviroment to Include the Package**
   ```bash
   source install/setup.sh
   ```

## Running the simulation
**The functionality of the simulation has been tested with a WSL Ubuntu-24.04 Image as well as on native Ubuntu-24.04 installation.**
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

## Troubleshooting
1. **MoveIt setup assistant crashes without explanation due to Rviz**:
This issue was seen when MoveIt setup assistant was used to initialize the MoveIt package.
For further info see this [**topic**](https://github.com/moveit/moveit2/issues/3546)
This issue could present itself if the user wants to modify the MoveIt configuration through the assistant.
2. **MoveIt Action Client not connected to Action Server**: 
This issue was seen when trying to perform MoveIt motion execution based on the planned paths.
The path planning would execute as expected but the motion could not be performed.
This was resolved by manually running:
```bash
sudo apt install ros-jazzy-moveit
```
again to fix some issue with the MoveIt Binary installation.
3. **Simulation does not load the FR3 robot arms**:
Remember to manually run
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/franka_ros2_ws/install/franka_description/share
```
Before launching the simulation.

## Contact
For questions or support regarding the package, please open an issue on the [GitHub Issues](https://github.com/AtteNyyssonen/robotics_project_work/issues) page.
