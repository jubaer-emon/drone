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
    dev_dir = Path(os.environ.get('DRONE_DEV_DIR', '~')).expanduser()
    pkg_dir = get_package_share_path('drone_sim')

    gz_resoure_path = ':'.join([
        str(pkg_dir / 'worlds'),
        str(pkg_dir / 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
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
            # 'HEADLESS': '1',
        },
        output='screen',
    )

    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # '/range@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )


    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_path('drone_bringup') / 'launch' / 'common.launch.py')
    )

    gcs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_path('drone_bringup') / 'launch' / 'gcs.launch.py')
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        gz_sim, 
        # ros_gz_bridge,
        
        px4_sitl,
        common_launch,
        gcs,
    ])