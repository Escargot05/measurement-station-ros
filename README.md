# Measurement Station

Author: [Dominik Zagórnik](https://github.com/Escargot05)

## Overview

The motivation behind this project was to optimize data collection from the environment with fiducial markers. The station consists of two main parts: a linearly moving carriage with an adapter for mounting sensors and a rotating profile for mounting markers. It is controlled by a ROS wrapper that sends G-code commands to a Melzi board (a basic Ender 3 3D printer motherboard) running modified Marlin firmware. 
The station server manages all sensors and the station itself, while the client records ROS topics into a bag file. These nodes can be run on separate devices. A typical station measurement scenario involves collecting x frames from the camera and y from the LiDAR at a given distance and marker rotation range in a stepwise manner.
The acquired data can be used for further research, processed using ROS packages, or exported to a more general format. This solution was designed in short time and to be cost-effective, and it succeeded quite well.

## Setup

### Install Dependencies

* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
* [rplidar package](https://github.com/Slamtec/rplidar_ros)
* [astra camera package](https://github.com/Escargot05/ros_astra_camera)
* serial package:

  ```
  sudo apt install ros-noetic-serial
  ```

Then, create a Catkin workspace and clone the repository:

```bash
mkdir -p ~/station_ws/src
cd ~/station_Ws/src
git clone https://github.com/Escargot05/measurement-station-ros
```

### Build

```bash
cd ~/station_ws
catkin_make
echo "source ~/station_ws/devel/setup.bash" >> ~/.bashrc
source ./deve/setup.bash
```

### Install udev Rules

```bash
roscd measurement_station
./scripts/create_udev_rules.sh
```

Remember to also install the udev rules for rplidar and the astra camera.

### rplidar

Edit the rplidar launch file to set the `serial_port` parameter.

```xml
<param name="serial_port" "type="string" value="/dev/rplidar"/>
```

## Launch

### Initialization

1. Set the path and name for the rosbag in `station_client.launch`.

2. Configure the measurement scenario in `station.yaml`.

3. Launch the station server and client:
  
   ```bash
   roslaunch measurement_station station_server.launch
   roslaunch measurement_Station station_cleint.launch 
   ```

### Client Key Inputs

* `e` - Enable station motors
* `d` - Disable station motors
* `l` - Rotate marker counterclockwise
* `h` - Rotate marker clockwise
* `s` - Start data collection and open the bag
* `p` - Finish data collection, close the rosbag, and stop the server

## Data Examination

For images and simple numeric messages, `rqt_bag` is sufficient. However, for scans and point clouds, use `rviz`.

```bash
rqt_bag <path_to_rosbag>
```

![station-censor](https://github.com/user-attachments/assets/d585f9a9-026a-41f5-8d52-2f9f7df0af1f)
