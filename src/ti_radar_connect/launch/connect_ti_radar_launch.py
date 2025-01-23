from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Define the launch argument for config_path
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='radar_0_IWR1843_demo.json',
        description='Path to the radar configuration file relative CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs/'
    )
    
    # Get the package share directory
    package_share_dir = get_package_share_directory('ti_radar_connect')
    config_file_path = LaunchConfiguration('config_file')
    config_directory_path = "src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs/"
    full_config_path = os.path.join(package_share_dir, config_directory_path, config_file_path)

    return LaunchDescription([
        config_file_arg,
        Node(
            package='ti_radar_connect',
            namespace='Radar_0',
            executable='ti_radar_connect',
            name='ti_radar_connect',
            output='screen',
            parameters=[{'config_path': full_config_path}]
        )
    ])