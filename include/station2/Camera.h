#ifndef CAMERA_H
#define CAMERA_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class Camera
{
private:
  ros::NodeHandle& nh_;
  ros::CallbackQueue queue_;

  ros::Subscriber info_sub_;
  ros::Subscriber color_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber ir_sub_;
  ros::Publisher info_pub_;
  ros::Publisher color_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher ir_pub_;
  

  static int image_count_;
  int image_number_;
  // int rate_;
  // bool store_data_;

  void infoCallback_(const sensor_msgs::CameraInfo::ConstPtr& info);
  void colorCallback_(const sensor_msgs::Image::ConstPtr& img);
  void depthCallback_(const sensor_msgs::Image::ConstPtr& img);
  void irCallback_(const sensor_msgs::Image::ConstPtr& img);

public:
  Camera(ros::NodeHandle& nh, std::string name);

  // void updateImageNumber(int scan_number);
  static void setImageCount(int image_count);
  static int getImageCount();
  void sendData();
};

#endif
