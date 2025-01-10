#using python3.10.5 at /bin/python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='ti_radar_connect',
                namespace='Radar_0',
                executable='ti_radar_connect',
                name='ti_radar_connect',
                output='screen',
                parameters=[
                    {'config_path':'/home/cpsl/Documents/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs/radar_0_IWR1843_demo.json'}
                ]
            )#,
            # Node(
            #     package='ti_radar_connect',
            #     namespace='Radar_1',
            #     executable='ti_radar_connect',
            #     name='ti_radar_connect',
            #     output='screen',
            #     parameters=[
            #         {'config_path':'/path/to/config_file'}
            #     ]
            # )
        ]
    )