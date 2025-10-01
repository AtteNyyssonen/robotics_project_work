from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('fr3_dual_arm_sim')
    world = os.path.join(pkg, 'worlds', 'simulation_world.sdf')
    urdf = os.path.join(pkg, 'urdf', 'dual_fr3.xacro')

    return LaunchDescription([
        # Launch Gazebo Sim
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world],
            output='screen'
        ),

        # Spawn the robot entity into Gazebo Sim
        #Node(
            #package='ros_gz_sim',
            #executable='create',
            #arguments=[
                #'-name', 'dual_fr3',
                #'-file', urdf
            #],
            #output='screen'
        #),
    ])
