from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path
from pathlib import Path
import os

def generate_launch_description():
    logger = get_logger('sim.launch')
    dev_dir = Path(__file__).parents[6].resolve()
    pkg_path = get_package_share_path('drone_bringup')

    gz_resoure_path = ':'.join([
        str(pkg_path / 'worlds'),
        str(pkg_path / 'models'),
        # os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])
 
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'simple_room.sdf'],
        additional_env= {
            'GZ_SIM_RESOURCE_PATH': gz_resoure_path,
        },
        output='screen',
    )

    px4_sitl = ExecuteProcess(
        cmd=['./build/px4_sitl_default/bin/px4'],
        cwd=dev_dir / 'PX4-Autopilot',
        additional_env= {
            'PX4_GZ_WORLD': 'simple_room',
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'x500_lidar_2d',
            'PX4_GZ_STANDALONE': '1',
            # 'GZ_SIM_RESOURCE_PATH': gz_resoure_path,
            # 'HEADLESS': '1',
        },
        output='screen',
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


    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path / 'launch' / 'common.launch.py'),
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

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        gz_sim,
        clock_bridge,
        lidar_bridge,
        
        px4_sitl,
        common_launch,
        # qgc,
        # rviz,
    ])