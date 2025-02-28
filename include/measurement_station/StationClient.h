#ifndef STATIONCLIENT_H_
#define STATIONCLIENT_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <termios.h>
#include <signal.h>
#include <vector>
#include <array>

#define KEYCODE_S 0x73
#define KEYCODE_P 0x70
#define KEYCODE_H 0x68
#define KEYCODE_L 0x6C
#define KEYCODE_E 0x65
#define KEYCODE_D 0x64

class StationClient
{
public:
  StationClient(ros::NodeHandle& nh, std::string lidar_name, std::string camera_name, std::string camera2_name);

  void getInput();

private:
  ros::NodeHandle& nh_;
  rosbag::Bag bag_;

  ros::Publisher key_code_;

  std::vector<ros::Subscriber> subscribers_;

  ros::Subscriber distance_;
  ros::Subscriber angle_;

  int getch_();
  void bagOpen_();
  void bagClose_();
  void sendKey_(int c);

  template <typename T>
  void callback_(const typename T::ConstPtr& msg, const std::string& topic);

  void distanceCallback_(const std_msgs::Int32::ConstPtr& num);
  void angleCallback_(const std_msgs::Int32::ConstPtr& num);

};

#endif
