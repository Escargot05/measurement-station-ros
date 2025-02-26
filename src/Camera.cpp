#include "measurement_station/Camera.h"

int Camera::image_count_;

Camera::Camera(ros::NodeHandle& nh, std::string name) : nh_(nh), image_number_(0)
{
  std::string cloud_topic;
  std::string color_topic;
  std::string depth_topic;
  std::string ir_topic;

  nh_.getParam("station/" + name + "/cloud_topic", cloud_topic);
  nh_.getParam("station/" + name + "/color_topic", color_topic);
  nh_.getParam("station/" + name + "/depth_topic", depth_topic);
  nh_.getParam("station/" + name + "/ir_topic", ir_topic);

  ros::SubscribeOptions ops1 = ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
      cloud_topic, 1, boost::bind(&Camera::cloudCallback_, this, _1), ros::VoidPtr(), &queue_);
  ros::SubscribeOptions ops2 = ros::SubscribeOptions::create<sensor_msgs::Image>(
      color_topic, 1, boost::bind(&Camera::colorCallback_, this, _1), ros::VoidPtr(), &queue_);
  ros::SubscribeOptions ops3 = ros::SubscribeOptions::create<sensor_msgs::Image>(
      depth_topic, 1, boost::bind(&Camera::depthCallback_, this, _1), ros::VoidPtr(), &queue_);
  ros::SubscribeOptions ops4 = ros::SubscribeOptions::create<sensor_msgs::Image>(
      ir_topic, 1, boost::bind(&Camera::irCallback_, this, _1), ros::VoidPtr(), &queue_);

  cloud_sub_ = nh_.subscribe(ops1);
  color_sub_ = nh_.subscribe(ops2);
  depth_sub_ = nh_.subscribe(ops3);
  ir_sub_ = nh_.subscribe(ops4);

  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name + "/cloud_data", 10);
  color_pub_ = nh_.advertise<sensor_msgs::Image>(name + "/color_data", 10);
  depth_pub_ = nh_.advertise<sensor_msgs::Image>(name + "/depth_data", 10);
  ir_pub_ = nh_.advertise<sensor_msgs::Image>(name + "/ir_data", 10);
}

void Camera::sendData()
{
  queue_.callAvailable();

  ROS_INFO("Image no: %d", image_number_);
  image_number_ = ++image_number_ % image_count_;
}

void Camera::setImageCount(int image_count)
{
  image_count_ = image_count;
}

int Camera::getImageCount()
{
  return image_count_;
}

void Camera::cloudCallback_(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  cloud_pub_.publish(cloud);
}

void Camera::colorCallback_(const sensor_msgs::Image::ConstPtr& img)
{
  color_pub_.publish(img);
}

void Camera::depthCallback_(const sensor_msgs::Image::ConstPtr& img)
{
  depth_pub_.publish(img);
}

void Camera::irCallback_(const sensor_msgs::Image::ConstPtr& img)
{
  ir_pub_.publish(img);
}
