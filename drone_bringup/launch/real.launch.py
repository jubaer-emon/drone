from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path

from pathlib import Path

def generate_launch_description():
    logger = get_logger('real.launch')

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('drone_bringup') 
                / 'launch' 
                / 'common.launch.py')
        )
    )

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
    )

    return LaunchDescription([
        common_launch,
        lidar,
    ])