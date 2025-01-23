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

## 1. Launching the ROS2 Node to stream data

If you want to collect a dataset which includes raw Radar data from a DCA1000 board, follow these instructions. **NOTE: ONLY THE IWR1443 + DCA1000 is supported. Future updates will support the IWR1843 and IWR6843**.

### Configuration files

There are three configuration files that must be loaded inorder for data capture with the DCA1000 to work correctly. Please review the following instructions carefully to ensure that everything is setup correctly. 

1. **IWR .cfg file**: First, you must generate a .cfg file for the TI IWR that you are using. This file includes all of the essential radar parameters which affect the radar's sensing specifications. Several examples are available in the [configurations/DCA1000/custom_configs](./radar_connect/include/CPSL_TI_Radar/configurations/DCA1000/custom_configs/) folder in the CPSL_TI_Radar module. More information on how to setup and analyze these configurations can be found on the [CPSL_TI_Radar c++ module documentation](https://github.com/davidmhunt/CPSL_TI_Radar/tree/main/CPSL_TI_Radar_cpp) under "Radar .cfg file".

2. **CPSL_TI_Radar_cpp .json configuration**: Next, there is a .json configuration that you will have to generate for each DCA1000 that you want to integrate with. Several examples are available in the [CPSL_TI_Radar_cpp/configs](./radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs/) of the CPSL_TI_Radar_cpp module code. **Pay close attention to the config path in this .json configuration for each DCA1000 as this is the configuration that will be loaded onto the IWR when running the ROS nodes** More information on how to setup and analyze these configurations can be found on the [CPSL_TI_Radar c++ module documentation](https://github.com/davidmhunt/CPSL_TI_Radar/tree/main/CPSL_TI_Radar_cpp) under "Radar .cfg file".

### Launching Radars [NOTE: need to confirm this]

An example launch file can be found in the [launch](./src/ti_radar_connect/launch/). Note, that each radar will need to launch its own ti_radar_connect node. To launch the sample node, simply run the following command to launch radar operations:
```
ros2 launch ti_radar_connect connect_ti_radar_launch.py config_file:=radar_0_IWR1843_demo.json
```