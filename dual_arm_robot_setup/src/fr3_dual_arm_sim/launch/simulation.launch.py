import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('fr3_dual_arm_sim')

    world = os.path.join(pkg, 'worlds', 'simulation_world.sdf')

    urdf_path = os.path.join(pkg, 'urdf', 'dual_fr3.xacro')


    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
              '-entity', 'dual_fr3',
              '-file', urdf_path,
              '-topic', 'robot_description',
              '-x', '0', '-y', '0', '-z', '0'
            ],
            output='screen'
        ),
    ])
