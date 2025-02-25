#ifndef STATIONSERVER_H_
#define STATIONSERVER_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "Camera.h"
#include "Lidar.h"
#include "StationController.h"

#define KEYCODE_S 0x73
#define KEYCODE_P 0x70
#define KEYCODE_H 0x68
#define KEYCODE_L 0x6C
#define KEYCODE_E 0x65
#define KEYCODE_D 0x64

class StationServer
{
public:
  StationServer(ros::NodeHandle& nh, std::string lidar_name, std::string camera_name);
  
  void performAction();

private:
  ros::NodeHandle& nh_;

  ros::Subscriber key_code_;

  Lidar rplidar_;
  Camera astra_;
  StationController station_;

  int key_;
  bool continous_measurement_;
  const double MEASUREMENT_TIME_;

  void keyCallaback_(const std_msgs::Int32::ConstPtr& num);
  void harvestData_();

};

#endif
