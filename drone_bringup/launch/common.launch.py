from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction 
from launch_ros.actions import Node
from launch.logging import get_logger

def generate_launch_description():
    logger = get_logger('common.launch')

    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='log',
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.12', '--y', '0', '--z', '0.26',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_link'
        ],
        output='screen',
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.1', '--y', '0', '--z', '0.2',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ],
        output='screen',
    )

    range_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.1', '--y', '0', '--z', '0.15',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'range_link'
        ],
        output='screen',
    )

    px4_odom = Node(
        package='drone_interface',
        executable='px4_odom_bridge',
        output='screen',
    )

    # slam = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[{
    #         'laser_frame': 'lidar_link',
    #         'odom_frame': 'odom',
    #         'base_frame': 'base_link',
    #         'map_frame': 'map',
    #         'odom_topic': '/odom',
    #         'scan_topic': '/scan',
    #         'scan_queue_size': 30,
    #         'transform_timeout': 2.0,
    #         'tf_buffer_duration': 10.0,
    #         'use_scan_matching': True,
    #         'use_scan_barycenter': True,
    #         'do_loop_closing': True,
    #         'minimum_travel_distance': 0.01,
    #         'minimum_travel_heading': 0.01,
    #         'throttle_scans': 1,
    #     }]
    # )

    return LaunchDescription([
        agent,

        # lidar_tf,
        # camera_tf,
        # range_tf,
        # px4_odom,

        # slam,
    ])