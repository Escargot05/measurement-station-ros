#ifndef STATIONCLIENT_H_
#define STATIONCLIENT_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <termios.h>
#include <signal.h>

#define KEYCODE_S 0x73
#define KEYCODE_P 0x70
#define KEYCODE_H 0x68
#define KEYCODE_L 0x6C
#define KEYCODE_E 0x65
#define KEYCODE_D 0x64

class StationClient
{
private:
  ros::NodeHandle& nh_;
  rosbag::Bag bag_;

  ros::Publisher key_code_;

  ros::Subscriber laser_;
  ros::Subscriber laser_corrected_;
  ros::Subscriber cloud_;
  ros::Subscriber cloud_corrected_;
  ros::Subscriber camera_info_;
  ros::Subscriber camera_color;
  ros::Subscriber camera_depth_;
  ros::Subscriber camera_ir_;
  ros::Subscriber distance_;
  ros::Subscriber angle_;

  void laserCallback_(const sensor_msgs::LaserScan::ConstPtr& scan);
  void laserCorrectedCallback_(const sensor_msgs::LaserScan::ConstPtr& scan);
  void cloudCallback_(const sensor_msgs::PointCloud::ConstPtr& cloud);
  void cloudCorrectedCallback_(const sensor_msgs::PointCloud::ConstPtr& cloud);
  void cameraInfoCallback_(const sensor_msgs::CameraInfo::ConstPtr& info);
  void cameraColorCallback_(const sensor_msgs::Image::ConstPtr& img);
  void cameraDepthCallback_(const sensor_msgs::Image::ConstPtr& img);
  void cameraIrCallback_(const sensor_msgs::Image::ConstPtr& img);
  void distanceCallback_(const std_msgs::Int32::ConstPtr& num);
  void angleCallback_(const std_msgs::Int32::ConstPtr& num);

public:
  StationClient(ros::NodeHandle& nh, std::string lidar_name, std::string lidar2_name, std::string camera_name);

  int getch();
  void bagOpen();
  void bagClose();
  void sendKey(int c);
};

#endif