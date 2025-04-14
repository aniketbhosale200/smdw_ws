from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Load and process the robot description
    robot_description_file = os.path.join(get_package_share_directory("agri_bot_moveit"), "config", "agri_bot.urdf")
    robot_description_config = xacro.process_file(robot_description_file)
    robot_desc = robot_description_config.toxml()

    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("agri_bot", package_name="agri_bot_moveit")
        .robot_description(file_path=robot_description_file)
        .robot_description_semantic(file_path="config/agri_bot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "verbose": "true",
            "pause": "false",
            "world": "/opt/ros/humble/share/gazebo_ros/worlds/empty.world"
        }.items(),
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare("agri_bot_moveit"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "agri_bot"],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Controller manager
    controllers_file = PathJoinSubstitution([FindPackageShare("agri_bot"), "config", "agri_bot_controller.yaml"])
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        output="screen",
    )

    # Joint state broadcaster (no delay)
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Arm controller (wait 3 seconds)
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_with_delay = TimerAction(period=3.0, actions=[arm_spawner])

    # Gripper controller (wait 6 seconds)
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    gripper_with_delay = TimerAction(period=6.0, actions=[gripper_spawner])

    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        joint_state_spawner,    # Starts immediately
        arm_with_delay,         # Starts after 3 seconds
        gripper_with_delay,     # Starts after 6 seconds
        move_group_node,
        rviz_node,
    ])