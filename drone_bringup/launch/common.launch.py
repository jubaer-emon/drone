from launch import LaunchDescription
from launch.actions import ExecuteProcess
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
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_link'
        ],
        output='screen'
    )

    px4_odom = Node(
        package='drone_interface',
        executable='px4_odom_bridge',
        output='screen'
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            # 'use_sim_time': 'true',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'scan_queue_size': 30,
            'transform_timeout': 2.0,
            'tf_buffer_duration': 10.0,

            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,

            'throttle_scans': 2,
        }]
    )

    return LaunchDescription([
        agent,

        lidar_tf,
        px4_odom,

        slam,
    ])