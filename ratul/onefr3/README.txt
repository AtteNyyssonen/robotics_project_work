How to build and launch the FR3 robot in MoveIt 2 (Jazzy)
=========================================================

1. Install ROS 2 Jazzy desktop and rosdep:
   sudo apt install ros-jazzy-desktop python3-rosdep -y
   sudo rosdep init
   rosdep update

2. Extract the workspace to your home folder:
   unzip onefr3.zip -d ~/

3. Build:
   cd ~/onefr3
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash

4. Launch the robot in RViz2:
   ros2 launch fr3_moveit2_custom demo.launch.py
