from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch system for ROS2 packages"""

    # ================ Launch arguments ==================
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run without GUI pose issuer'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation time and sim-specific parameters'
    )

    testing_arg = DeclareLaunchArgument(
        'testing',
        default_value='false',
        description='Enable testing mode with additional logging'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Robot namespace/name'
    )

    headless = LaunchConfiguration('headless')
    sim = LaunchConfiguration('sim')
    testing = LaunchConfiguration('testing')
    robot_name = LaunchConfiguration('robot_name')

    # ================ Simulation settings ==================

    # Turtlesim node (only runs if sim==true)
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='simulator',
        condition=IfCondition(sim),
        remappings=[
            ('/turtle1/cmd_vel', '/cmd_vel'),
        ],
        output='screen'
    )

    # Turtlesim Odometry Bridge Node (only runs if sim==true)
    turtlesim_odom_bridge_node = Node(
        package='turtlesim_pose_publisher',
        executable='turtlesim_pose_publisher',
        name='turtlesim_pose_publisher',
        condition=IfCondition(sim),
        output='screen'
    )

    # Turtlesim Velocity Bridge Node
    turtlesim_velocity_transformer = Node(
        package='turtlesim_velocity_transformer',
        executable='turtlesim_velocity_transformer',
        name='turtlesim_velocity_transformer',
        output='screen'
    )

    # ================ ROS 2 Packages ==================

    # Clock pose issuer node (always runs)
    clock_pose_issuer_node = Node(
        package='clock_pose_issuer',
        executable='clock_node',
        name='clock_pose_issuer',
        parameters=[{
            # 'use_sim_time': sim,
            'publish_rate': 10.0,
            'frame_id': 'map',
            'topic_name': '/target_pose_clock',
        }],
        output='screen'
    )

    # Motion controller node (always runs)
    motion_controller_node = Node(
        package='motion_controller',
        executable='motion_controller',
        name='motion_controller',
        parameters=[{
            # 'use_sim_time': sim,
            'robot_name': robot_name,
            'frame_id': 'map',
        }],
        output='screen'
    )

    # ================ Launch description ==================
    return LaunchDescription([
        headless_arg,
        sim_arg,
        testing_arg,
        robot_name_arg,
        turtlesim_node,
        turtlesim_odom_bridge_node,
        turtlesim_velocity_transformer,
        clock_pose_issuer_node,
        motion_controller_node,
    ])