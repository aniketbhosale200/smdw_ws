import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration, NotSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    right_smdw_dir = get_package_share_directory("right_smdw")
    
    # Declare argument for model path
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(right_smdw_dir, "urdf", "right_smdw.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )
    
    # Declare argument for RViz config file
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(right_smdw_dir, "rviz", "right_smdw_config.rviz"),
        description="Absolute path to RViz configuration file"
    )
    
    # Declare world file argument
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(right_smdw_dir, "worlds", "empty.world"),
        description="Path to world file"
    )
    
    # Declare sim time parameter
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time"
    )
    
    # Declare GUI parameter for joint state publisher
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Start Joint State Publisher GUI"
    )
    
    # Declare RViz launch parameter
    rviz_arg = DeclareLaunchArgument(
        name="rviz",
        default_value="true",
        description="Start RViz"
    )
    
    # Generate robot_description from Xacro 
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )
    
    # Start robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }]
    )
    
    # Start joint_state_publisher (only if GUI is not used)
    joint_state_publisher_node = Node(
        condition=UnlessCondition(LaunchConfiguration("gui")),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )
    
    # Start joint_state_publisher_gui (if GUI is enabled)
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(LaunchConfiguration("gui")),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )
    
    # Set Gazebo model path
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.join(right_smdw_dir, "models")
    )
    
    # Form the path to the Gazebo Classic launch file from gazebo_ros package
    gazebo_launch_path = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )
    
    # Include Gazebo Classic simulation launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            "verbose": "true",
            "world": LaunchConfiguration("world")
        }.items()
    )
    
    # Spawn the robot in Gazebo Classic using spawn_entity.py (delayed to ensure robot description is ready)
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "/robot_description", "-entity", "right_smdw"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    # Delay spawn_entity_node by 5 seconds
    spawn_entity = TimerAction(
        period=5.0,
        actions=[spawn_entity_node]
    )
    
    # Launch RViz conditionally
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        world_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_model_path,
        gazebo,
        spawn_entity,
        rviz_node
    ])
