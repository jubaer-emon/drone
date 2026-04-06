from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path
from pathlib import Path

def generate_launch_description():
    logger = get_logger('gcs.launch')

    qgc = ExecuteProcess(
        cmd=['./QGroundControl-x86_64.AppImage'],
        cwd=Path.home() / 'Downloads',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', str(get_package_share_path('nav2_bringup') / 'rviz' / 'nav2_default_view.rviz'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        rviz,
        qgc,
    ])