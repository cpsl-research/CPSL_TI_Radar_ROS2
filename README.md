# CPSL_TI_Radar_ROS
Set of ROS packages which can be used to integrate with the CPSL_TI_Radar C++ codebase available at the [CPSL_TI_Radar Repository](https://github.com/davidmhunt/TI_Radar_Demo_Visualizer). 
## Installation (Ubuntu 24.04/ ROS2 Jazzy):

### Install ROS2
1. Follow the instructions on the [ROS2 Jazzy installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) website to install ROS. If you are unfamiliar with ROS2, its worth taking some of the [ROS2 Jazzy Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)

### Adding CPSL_TI_Radar_ROS2 packages to catkin workspace
We provide several ROS2 packages to integrate with the CPSL_TI_Radar module:
* ti_radar_connect: a ROS package that handles connecting to either the TI-1443, TI-IWR1843, TI-IWR6843, and/or DCA1000 to receive either point cloud TLV packets or raw ADC datacubes from the radar
* radar_msgs: a ROS package defining custom messages used for topics on the adc data cube, range azimuth response, range doppler response, and the processed radar point cloud
* radar_sig_processing [coming soon]: a set of ROS packages used for receiving the range doppler and range azimuth responses from the CPSL_TI_Radar module
* radar_views [coming soon]: a ROS pacakge used for viewing the range-azimuth and range-doppler responses in windows. Additionally includes functionality for viewing the rad-nav model's output in rviz.

1. To add the packages to an existing colcon workspace, perform the following commands
```
cd [colcon_ws]/src/
git clone --recurse-submodules https://github.com/cpsl-research/CPSL_TI_Radar_ROS2.git
```
Here, [colcon_ws] is the path to your colcon workspace

If you forgot to perform the --recurse-submodules when cloning the repository, you can use the following command to load the necessary submodules
```
git submodule update --init --recursive
```

2. Since this package utilizes c++ to interact with the DCA1000, we must make sure that all of the pre-requisites are satisfied. To ensure that all pre-requisites are satisfied, please follow the "Pre-requisite packages" instructions in the [CPSL_TI_Radar_cpp github installation instructions](https://github.com/davidmhunt/CPSL_TI_Radar/tree/main/CPSL_TI_Radar_cpp)


3. Next, install all of the required ros dependencies using the following command
```
cd ~/[colcon_ws]
rosdep install --from-paths src --rosdistro jazzy -y
```

4. Next, build the ROS nodes in your catkin workspace using the following commands:
```
cd ~/[colcon_ws]
colcon build --symlink-install
```

5. Finally, source the setup.bash file so that ROS can find the nodes and the messages
```
source install/setup.bash
```

# Tutorials

Below are several tutorials for utilizing the CPSL_TI_Radar ROS2 nodes and integrating them with other sensor measurements. For these tutorials, its often helpful to use a terminal with multiple windows. Here, we recommend using a terminal viewer like [tmux](https://tmuxcheatsheet.com/#:~:text=Tmux%20Cheat%20Sheet%20%26%20Quick%20Reference%201%20Sessions,6%20Help%20%24%20tmux%20list-keys%20%3A%20list-keys%20) which allows you to create/view multiple panes and windows in a single terminal window. 

## 1. Streaming point cloud data from IWR1843/IWR6843 Radar boards

Follow these instructions to stream raw radar point clouds from an IWR1843 or IWR6843 radar board in ROS2

1. **Radar Hardware Setup**. Before continuing, follow the steps in the [CPSL_TI_Radar_cpp Readme](./src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/Readme.md). The current code is built for mmWave SDK3.6, but it should also work on other datasets as well.

2. **IWR .cfg file**: Next, setup a .cfg file for the TI IWR that you are using. This file includes all of the essential radar parameters which affect the radar's sensing specifications. Many samples are available in the [CPSL_TI_Radar/configurations](./src/ti_radar_connect/include/CPSL_TI_Radar/configurations/). Additional configurations can also be generated using the Ti mmWave Demo if desired as well.

3. **.json configuration file**: Next, setup a .json configuration file that is used to actually run the CPSL_TI_Radar_cpp code used by the ROS2 nodes. Example json configs can be found in the [CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs](./src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs). If just streaming the raw data (i.e.; not ADC samples using the DCA1000), be sure to set the DCA1000_streaming.enabled component to false.

4. **Build and install ROS2 nodes**: Once setup, rebuild and install the ROS2 nodes
```
cd CPSL_TI_Radar_ROS2
colcon build --symlink-install
```

5. Finally, each radar can be launched using the following launch command
```
ros2 launch ti_radar_connect connect_ti_radar_launch.py config_file:=radar_0_IWR1843_nav.json namespace:=radar_0
```
The command has the following parameters

| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `config_file`   | radar_0_IWR1843_demo.json  | the .json config file path in the CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs |  
| `namespace`| Radar_0| the namespace to use when publishing the point cloud and the frame id of the point cloud