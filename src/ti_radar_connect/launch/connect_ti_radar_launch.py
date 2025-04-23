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
    DeclareLaunchArgument(
        'stamp_delay_sec', default_value='0.00',
        description='delay to applied to the published radar messages time'),
    DeclareLaunchArgument(
        'tf_prefix', default_value='',
        description='tf_prefix to apply to frame id')
]

def launch_setup(context,*args,**kwargs):
    frame_id = LaunchConfiguration('frame_id')
    config_file = LaunchConfiguration('config_file')
    stamp_delay_sec = LaunchConfiguration('stamp_delay_sec')
    tf_prefix = LaunchConfiguration('tf_prefix')

    #derive the full config path
    config_file_str = config_file.perform(context)
    package_share_dir = get_package_share_directory('ti_radar_connect')
    config_directory_path = "configs"
    full_config_path = os.path.join(package_share_dir, config_directory_path, config_file_str)

    #load the tf prefix
    tf_prefix_str = tf_prefix.perform(context)
    frame_id_str = frame_id.perform(context)
    if (tf_prefix_str):
        frame_id_str = "/{}/{}".format(tf_prefix_str,frame_id_str)

    nodes = GroupAction([
        Node(
            package='ti_radar_connect',
            executable='ti_radar_connect',
            name='ti_radar_connect',
            output='screen',
            parameters=[{'config_path': full_config_path,
                         'frame_id':frame_id_str,
                         'stamp_delay_sec':stamp_delay_sec}]
        )
    ])

    return [nodes]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
