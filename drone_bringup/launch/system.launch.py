from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path
from launch.logging import get_logger

def generate_launch_description():
    logger = get_logger('drone_bringup')
    
    dev_dir = Path(__file__).parents[6].resolve()
    px4_path = dev_dir / 'PX4-Autopilot'
    qgc_path = dev_dir / 'QGroundControl-x86_64.AppImage'

    use_sim = LaunchConfiguration('use_sim')

    # ---- ARG ----
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Run in simulation mode'
    )

    set_sim_time = SetParameter(
        name='use_sim_time',
        value=ParameterValue(use_sim, value_type=bool)
    )

    # ---- MicroXRCEAgent (always) ----
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='log',
    )

    # ---- PX4 SITL (only sim) ----
    px4_sitl = ExecuteProcess(
        cmd=['./build/px4_sitl_default/bin/px4'],
        cwd=px4_path,
        additional_env= {
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'gz_x500_lidar_2d_modified',
            'PX4_GZ_WORLD': 'walls',
        },
        output='screen',
        condition=IfCondition(use_sim),
    )

    # ---- QGroundControl (optional sim) ----
    qgc = ExecuteProcess(
        cmd=['./QGroundControl-x86_64.AppImage'],
        cwd=dev_dir,
        output='screen',
        condition=IfCondition(use_sim)
    )

    # ---- CLOCK BRIDGE (Gazebo → ROS) ----
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
        condition=IfCondition(use_sim)
    )

    # ---- TF: base_link → lidar_link ----
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

    # ---- LiDAR ----
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        condition=IfCondition(use_sim)
    )

    lidar_real = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        condition=UnlessCondition(use_sim)
    )

    # ---- PX4 → ODOM bridge (always) ----
    wait_px4 = Node(
        package='drone_interface',
        executable='wait_for_px4',
        output='screen'
    )
    px4_odom = Node(
        package='drone_interface',
        executable='px4_odom_bridge',
        output='screen'
    )

    # ---- SLAM (delayed after TF & LiDAR) ----
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
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
    
    start_px4_odom = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_px4,
            on_exit=[px4_odom]
        )
    )

    start_slam = TimerAction(
        period=15.0,
        actions=[slam]
    )

    # ---- RViz (optional last) ----
    rviz = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        )]
    )

    return LaunchDescription([
        declare_use_sim,
        set_sim_time,

        agent,
        px4_sitl,
        qgc,
        clock_bridge,

        lidar_tf,
        lidar_bridge,
        lidar_real,
        wait_px4,
        start_px4_odom,
        start_slam,

        rviz,
    ])