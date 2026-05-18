# CPSL_TI_Radar_ROS2

ROS2 packages for integrating TI mmWave radars (IWR1443, IWR1843, IWR6843) with the [CPSL_TI_Radar C++ library](https://github.com/davidmhunt/CPSL_TI_Radar). Supports both serial TLV point-cloud streaming and raw ADC capture via a DCA1000 capture card.

## Packages

| Package | Description |
|---------|-------------|
| `raw_radar_msgs` | Custom message definitions: `ADCDataCube` (raw ADC samples as int16 arrays) |
| `ti_radar_connect` | Driver node — connects to IWR radar and/or DCA1000, publishes point clouds or ADC cubes |

## Prerequisites

1. **C++ library dependencies** — follow the "Pre-requisite packages" instructions in the [CPSL_TI_Radar_cpp README](./src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/Readme.md).

2. **Serial port access** — add your user to the `dialout` group (log out and back in after):
   ```bash
   sudo usermod -a -G dialout $USER
   ```

3. **DCA1000 high-rate streaming** — raise the kernel UDP receive buffer cap before running:
   ```bash
   sudo sysctl -w net.core.rmem_max=134217728
   ```
   To make permanent:
   ```bash
   echo 'net.core.rmem_max=134217728' | sudo tee /etc/sysctl.d/99-radar.conf
   sudo sysctl -p /etc/sysctl.d/99-radar.conf
   ```

## Build

This package is used as part of the [CPSL_ROS2_Sensors](https://github.com/cpsl-research/CPSL_ROS2_Sensors) workspace. Follow the build instructions in that repo's README. Build these packages first:

```bash
eval $(poetry env activate)
python -m colcon build --packages-select raw_radar_msgs --symlink-install
python -m colcon build --packages-select ti_radar_connect --symlink-install
source install/setup.bash
```

## Launch

Each radar is launched individually via `connect_ti_radar_launch.py`:

```bash
ros2 launch ti_radar_connect connect_ti_radar_launch.py \
    config_file:=radar_0_IWR1843_vel_sr.json \
    radar_name:=radar_0 \
    tf_prefix:=cpsl_ugv_1 \
    stamp_delay_sec:=0.1
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_file` | `radar_0_IWR1843_demo.json` | Bare filename from `config/system/` |
| `radar_name` | `Radar_0` | Topic namespace and TF frame ID |
| `stamp_delay_sec` | `0.0` | Header timestamp offset in seconds |
| `tf_prefix` | `''` | Prefix prepended to the TF frame ID |

In practice, radars are launched from a higher-level bringup file (e.g. `ugv_sensor_bringup.launch.py`) rather than directly.

## Topics Published

All topics are scoped under `<radar_name>/`:

| Topic | Type | Description |
|-------|------|-------------|
| `<radar_name>/detected_points` | `sensor_msgs/PointCloud2` | Detected point cloud (serial streaming) |
| `<radar_name>/adc_data_cube` | `raw_radar_msgs/ADCDataCube` | Raw ADC samples (DCA1000 only) |
| `<radar_name>/radar_config_path` | `std_msgs/String` | Path to the active JSON config |

## Configuration Files

Two files are needed for each radar:

### 1. JSON system config (`config/system/*.json`)

Specifies serial ports, DCA1000 network settings, streaming mode, and points to the radar `.cfg` file:

```json
{
    "verbose": false,
    "TI_Radar_Config_Management": {
        "TI_Radar_config_path": "../radar/nav_configs/1843_vel_sr.cfg"
    },
    "CLI_Controller": {
        "CLI_port": "/dev/ttyACM0"
    },
    "Streamer": {
        "serial_streaming": {
            "enabled": true,
            "data_port": "/dev/ttyACM1"
        },
        "DCA1000_streaming": {
            "enabled": false,
            "FPGA_IP": "192.168.33.180",
            "system_IP": "192.168.33.30",
            "data_port": 4098,
            "cmd_port": 4096
        },
        "save_to_file": false,
        "board_type": "IWR1843"
    }
}
```

`TI_Radar_config_path` accepts **relative paths** (resolved relative to the JSON file's location) or absolute paths. All configs in `config/system/` use portable relative paths of the form `../radar/<subdir>/<file>.cfg`.

### 2. Radar `.cfg` file (`config/radar/`)

TI mmWave SDK chirp configuration. Must include `lvdsStreamCfg -1 0 1 0` when using DCA1000. Sample configs are organized under `config/radar/nav_configs/`, `config/radar/DCA1000/`, etc.

## Coordinate Frame Note

TI radars output point clouds in **East-North-Up (ENE)** convention. A 90° rotation is required to align with the ROS standard **Forward-Left-Up (FLU)** frame. This rotation is defined in the platform URDF (under `platform_descriptions/urdf/`), not in this driver.
