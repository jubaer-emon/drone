from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
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
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        str(dev_dir / 'PX4-Autopilot/Tools/simulation/gz/models'),
        str(dev_dir / 'PX4-Autopilot/Tools/simulation/gz/worlds'),
        str(pkg_dir / 'models'),
        str(pkg_dir / 'worlds'),
    ])

    px4_gz_plugins = str(dev_dir 
                         / 'PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins')
    
    gz_server_config = str(dev_dir 
                           / 'PX4-Autopilot/src/modules/simulation/gz_bridge/server.config')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'default.sdf', '--verbose', '4'],
        additional_env= {
            'PX4_GZ_WORLD': 'default',
            'PX4_GZ_PLUGINS': px4_gz_plugins,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': px4_gz_plugins,
            'GZ_SIM_RESOURCE_PATH': gz_resoure_path,
            'GZ_SIM_SERVER_CONFIG_PATH': gz_server_config,
            # 'DRI_PRIME': '1',
            # 'HEADLESS': '1',
        },
        output='screen',
    )

    px4_sitl = ExecuteProcess(
        cmd=['./build/px4_sitl_default/bin/px4'],
        cwd=str(dev_dir / 'PX4-Autopilot'),
        additional_env= {
            'PX4_GZ_WORLD': 'default',
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'x500_flow',
            'PX4_GZ_STANDALONE': '1',
        },
        output='screen',
    )

    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # '/fmu/in/vehicle_optical_flow@px4_msgs/msg/VehicleOpticalFlow[gz.msgs.CameraHomography',
            # '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # '/fmu/in/distance_sensor@px4_msgs/msg/DistanceSensor[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    # OPTIONAL: Uncomment this node if you ever want to bridge cameras cleanly
    # ros_gz_image_bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=['/camera/image_raw'],
    #     output='screen',
    # )

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('drone_bringup') 
                / 'launch' 
                / 'common.launch.py')
        )
    )

    gcs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('drone_bringup') 
                / 'launch' 
                / 'gcs.launch.py')
        )
    )

    shutdown_on_gz_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_sim,
            on_exit=[Shutdown(reason='Gazebo Sim exited or crashed.')]
        )
    )
    shutdown_on_px4_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=px4_sitl,
            on_exit=[Shutdown(reason='PX4 Autopilot SITL exited or crashed.')]
        )
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        gz_sim, 
        ros_gz_bridge,
        
        px4_sitl,
        common_launch,
        # gcs,

        shutdown_on_gz_exit,
        shutdown_on_px4_exit,
    ])