import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
import xacro

def generate_launch_description():
    pkg_my_robot_description = get_package_share_directory('fr3_dual_arm_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path_arg = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(pkg_my_robot_description, 'worlds', 'simulation_world.sdf'),
        description='Path to the Gazebo world file (.sdf)'
    )

    # --- Robot Description ---
    xacro_file = os.path.join(pkg_my_robot_description, 'urdf', 'dual_fr3.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Gazebo Sim ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [LaunchConfiguration('world_path'), ' -r -v 4']}.items(),
    )

    # --- Core Nodes ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'dual_fr3'],
        output='screen'
    )

    # --- ROS-Gazebo Bridge ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/dual_fr3/joint_state_broadcaster/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_path_arg,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        spawn_entity,
    ])
