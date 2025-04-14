from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for target position
    target_x_arg = DeclareLaunchArgument(
        'target_x',
        default_value='5.5',
        description='X-coordinate of the target position (in meters)'
    )
    
    target_y_arg = DeclareLaunchArgument(
        'target_y',
        default_value='4.0',
        description='Y-coordinate of the target position (in meters)'
    )

    # Define the move_to_point node
    move_to_point_node = Node(
        package='smdw_robot_base',
        executable='move_to_point',
        name='move_to_point_node',
        output='screen',  # Print logs to console for debugging
        parameters=[
            {'target_x': LaunchConfiguration('target_x')},
            {'target_y': LaunchConfiguration('target_y')},
            {'use_sim_time': True}  # Use Gazebo simulation time
        ]
    )

    # Return the launch description
    return LaunchDescription([
        target_x_arg,
        target_y_arg,
        move_to_point_node
    ])