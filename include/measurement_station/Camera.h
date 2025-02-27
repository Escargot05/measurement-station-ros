#ifndef CAMERA_H
#define CAMERA_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

class Camera
{
public:
  Camera(ros::NodeHandle& nh, std::string name);

  static void setImageCount(int image_count);
  static int getImageCount();
  void sendData();

protected:
  ros::NodeHandle& nh_;
  ros::CallbackQueue queue_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber color_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber ir_sub_;
  ros::Publisher cloud_pub_; 
  ros::Publisher color_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher ir_pub_;

  static int image_count_;
  int image_number_;

  void cloudCallback_(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void colorCallback_(const sensor_msgs::Image::ConstPtr& img);
  void depthCallback_(const sensor_msgs::Image::ConstPtr& img);
  void irCallback_(const sensor_msgs::Image::ConstPtr& img);
};

#endif
