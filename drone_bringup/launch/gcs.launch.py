from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path
from pathlib import Path

def generate_launch_description():
    logger = get_logger('gcs.launch')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    qgc = ExecuteProcess(
        cmd=['./QGroundControl-x86_64.AppImage'],
        cwd=str(Path.home() / 'Downloads'),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', 
            str(get_package_share_path('nav2_bringup') 
                / 'rviz' 
                / 'nav2_default_view.rviz'),
        ],
        additional_env={
            # 'DRI_PRIME': '1', 
        },
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        
        rviz,
        qgc,
    ])