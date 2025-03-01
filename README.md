# Measurement station

Author: [Dominik Zagórnik](https://github.com/Escargot05)

## Overview

The motivation behind this project was to optimize data collection from the environment with fiducial markers. The station consist of two main parts: linear moving carriage with adapter for mounting sensors, and rotating profile for mounting markers. It is controlled by ROS wrapper on G-code commands send to Melzi board (basic Ender 3 3D printer motherboard) with modified Marlin firmware. The station server controls all the sensors and station itself, and the client records the ROS topics into the bag. These nodes could be run on separate devices. The typicall station measurement scenarion is to collect the x frames from camera nad y from lidar on given distance and marker rotation ranges in stepwise manner. The aquired data can be used in further research, utilizing ROS packages or just be exported to some more general format. TBH, this solution was meant to be done fast and cheap, and it suceeded quite well.

## Set-up

### Install dependencies

* ROS Noetic
* rplidar package
* astra camera package
* serial package:

  ```
  sudo apt install ros-noetic-serial
  ```

Then create catkin workspace and clone the repo.

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

### Instal udev rules

```bash
roscd measurement_station
./scripts/create_udev_rules.sh
```

Remember to also install udev rules of rplidar and astra camera.

### rplidar

Edit rplidar launch file `serial_port` parameter.

```bash
<param name="serial_port" "type="string" value="/dev/rplidar"/>
```

## Launch

### Initialization

Set path and name for rosbag in `station_client.launch`

Set configuration for measurement scenario in `station.yaml`

Then launch station server and client.

```bash
roslaunch measurement_station station_server.launch
roslaunch measurement_Station station_cleint.launch 
```

### Client key inputs

* `e` enable station motors
* `d` disable station motors
* `l` rotate marker counter-clockwise
* `h` rotate marker clockwise
* `s` start data collection and open bag
* `p` finish data collection, close rosbag, and stop server

## Data examination

For images and simple numeric message `rqt_bag` is enough, but for scans and point clouds use `rviz`.

```bash
rqt_bag <path_to_rosbag>
```
