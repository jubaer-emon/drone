from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource

from pathlib import Path

def generate_launch_description():
    logger = get_logger('sim.launch')
    dev_dir = Path(__file__).parents[6].resolve()
    current_dir = Path(__file__).parent

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(current_dir / 'common.launch.py'),
    )
 
    set_sim_time = SetParameter(
        name='use_sim_time',
        value=True,
    )

    px4_sitl = ExecuteProcess(
        cmd=['./build/px4_sitl_default/bin/px4'],
        cwd=dev_dir / 'PX4-Autopilot',
        additional_env= {
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'gz_x500_lidar_2d_modified',
            'PX4_GZ_WORLD': 'walls',
        },
        output='screen',
    )

    qgc = ExecuteProcess(
        cmd=['./QGroundControl-x86_64.AppImage'],
        cwd=dev_dir,
        output='screen',
    )

    rviz = ExecuteProcess(
        cmd=['rviz2'],
        output='screen'
    ) 

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    return LaunchDescription([
        set_sim_time,

        common_launch,
        px4_sitl,

        # qgc,
        # rviz,

        clock_bridge,
        lidar_bridge,
    ])