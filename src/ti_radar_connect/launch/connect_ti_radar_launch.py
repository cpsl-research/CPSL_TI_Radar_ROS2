from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

ARGUMENTS = [
    DeclareLaunchArgument(
        'config_file',
        default_value='radar_0_IWR1843_demo.json',
        description='Path to the radar configuration file install/ti_radar_connect/share/ti_radar_connect/configs folder'
    ),
    DeclareLaunchArgument(
        'frame_id', default_value='Radar_0',
        description='radar frame_id'),
]

def launch_setup(context,*args,**kwargs):
    frame_id = LaunchConfiguration('frame_id')
    config_file = LaunchConfiguration('config_file')

    #derive the full config path
    config_file_str = config_file.perform(context)
    package_share_dir = get_package_share_directory('ti_radar_connect')
    config_directory_path = "configs"
    full_config_path = os.path.join(package_share_dir, config_directory_path, config_file_str)

    nodes = GroupAction([
        Node(
            package='ti_radar_connect',
            executable='ti_radar_connect',
            name='ti_radar_connect',
            output='screen',
            parameters=[{'config_path': full_config_path,
                         'frame_id':frame_id}]
        )
    ])

    return [nodes]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
