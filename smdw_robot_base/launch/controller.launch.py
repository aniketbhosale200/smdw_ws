import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory and controller config path
    robot_base_pkg = get_package_share_directory('smdw_robot_base')
    controller_config = os.path.join(robot_base_pkg, 'config', 'controller.yaml')
    
    # Validate config file exists
    if not os.path.exists(controller_config):
        raise FileNotFoundError(f"Controller configuration not found: {controller_config}")

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Controller manager node with detailed configuration
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description_topic': '/robot_description'}
        ],
        output='screen',
        on_exit=None  # Prevent automatic exit if there's an error
    )

    # Spawner for joint_state_broadcaster with extended timeout and error handling
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 
             'joint_state_broadcaster', 
             '--controller-manager-timeout', '60',
             '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawner for mecanum_drive_controller with extended timeout
    mecanum_drive_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 
             'mecanum_drive_controller', 
             '--controller-manager-timeout', '60',
             '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Ensure mecanum controller spawns after joint state broadcaster
    mecanum_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner]
        )
    )

    return LaunchDescription([
        use_sim_time,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        mecanum_controller_event_handler
    ]) 