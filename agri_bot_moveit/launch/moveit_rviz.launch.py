from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Load the simplified URDF (not Xacro, since it’s already processed without rollers)
    robot_description_file = os.path.join(
        get_package_share_directory("agri_bot"),
        "urdf",
        "agri_bot.urdf"
    )
    with open(robot_description_file, 'r') as file:
        robot_desc = file.read()

    # MoveIt config with simplified URDF
    moveit_config = (
        MoveItConfigsBuilder("agri_bot", package_name="agri_bot_moveit")
        .robot_description(file_path=robot_description_file)  # Use simplified URDF
        .robot_description_semantic(file_path="config/agri_bot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # MoveIt move_group node with explicit robot_description parameter
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},  # Pass simplified URDF directly
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz with explicit robot_description parameter
    rviz_config_file = PathJoinSubstitution([FindPackageShare("agri_bot_moveit"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"robot_description": robot_desc},  # Pass simplified URDF directly
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # Optional: Robot state publisher for MoveIt (only if needed, can be omitted if Gazebo’s is sufficient)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        move_group_node,
        rviz_node,
        robot_state_publisher,  # Comment out if Gazebo’s robot_state_publisher is sufficient
    ])