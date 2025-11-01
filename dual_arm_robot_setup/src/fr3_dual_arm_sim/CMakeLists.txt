cmake_minimum_required(VERSION 3.8)
project(fr3_dual_arm_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(xacro REQUIRED)

# FR3 / Franka dependencies
find_package(franka_description REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(franka_gripper REQUIRED)
find_package(franka_example_controllers REQUIRED)
find_package(franka_semantic_components REQUIRED)

# MoveIt / planning dependencies
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)
find_package(rviz2 REQUIRED)

install(DIRECTORY launch worlds models urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()