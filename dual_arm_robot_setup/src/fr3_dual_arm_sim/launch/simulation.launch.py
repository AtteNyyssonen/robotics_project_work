import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

def generate_launch_description():
    pkg_my_robot_description = get_package_share_directory('fr3_dual_arm_sim')
    pkg_moveit_description = get_package_share_directory('fr3_dual_arm_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path_arg = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(pkg_my_robot_description, 'worlds', 'simulation_world.sdf'),
        description='Path to the Gazebo world file (.sdf)'
    )

    xacro_file = os.path.join(pkg_moveit_description, 'config', 'dual_fr3.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Path to the SRDF file
    srdf_file = os.path.join(pkg_moveit_description, 'config', 'dual_fr3.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Path to MoveIt controllers config
    moveit_controllers_file = os.path.join(pkg_moveit_description, 'config', 'moveit_controllers.yaml')
    with open(moveit_controllers_file, 'r') as f:
        moveit_controllers_config = yaml.safe_load(f)

    kinematics_file_path = os.path.join(pkg_moveit_description, 'config', 'kinematics.yaml')
    with open(kinematics_file_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    kinematics_config_wrapped = {"robot_description_kinematics": kinematics_config}

    ompl_planning_file_path = os.path.join(pkg_moveit_description, 'config', 'ompl_planning.yaml')
    with open(ompl_planning_file_path, 'r') as f:
        ompl_planning_config = yaml.safe_load(f)
        
    ros2_controllers_file = os.path.join(pkg_moveit_description, 'config', 'ros2_controllers.yaml')

    rviz_config_file = os.path.join(pkg_moveit_description, 'config', 'moveit.rviz')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [LaunchConfiguration('world_path'), ' -r -v 4', ' --physics-engine gz-physics-bullet-featherstone-plugin']}.items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, robot_description_semantic]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_file,
        ],
        output="screen",
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'dual_fr3'],
        output='screen'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config_wrapped,
        ],
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_controllers_config,
            ompl_planning_config,
            kinematics_config_wrapped,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # --- ros2_control Controllers ---
    # These spawner nodes will load the controllers defined in your ros2_controllers.yaml
    # We use RegisterEventHandler to ensure they start *after* the robot is spawned.

    def create_spawner_node(controller_name):
        return ExecuteProcess(
            cmd=[[
                FindExecutable(name="ros2"),
                " run ",
                "controller_manager",
                " spawner ",
                controller_name,
                " --controller-manager-timeout 300",
                " --controller-manager /controller_manager",
            ]],
            shell=True,
            output="screen",
        )

    spawn_jsb = create_spawner_node("joint_state_broadcaster")
    spawn_left_arm = create_spawner_node("left_fr3_arm_controller")
    spawn_right_arm = create_spawner_node("right_fr3_arm_controller")
    spawn_left_hand = create_spawner_node("left_fr3_hand_controller")
    spawn_right_hand = create_spawner_node("right_fr3_hand_controller")

    spawn_other_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_left_arm, spawn_right_arm, spawn_left_hand, spawn_right_hand],
        )
    )

    spawn_jsb_event = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_jsb],
        )
    )

    return LaunchDescription([
        world_path_arg,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        controller_manager_node,
        spawn_entity,
        rviz_node,
        move_group_node,
        spawn_jsb_event,
        spawn_other_controllers,
    ])
