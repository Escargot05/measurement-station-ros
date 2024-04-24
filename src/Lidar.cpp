#include "station2/Lidar.h"

int Lidar::scan_count_;

Lidar::Lidar(ros::NodeHandle& nh, std::string name) : nh_(nh), scan_number_(0)
{
  std::string topic;

  nh_.getParam("station/" + name + "/topic", topic);

  ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
      topic, 1, boost::bind(&Lidar::callback_, this, _1), ros::VoidPtr(), &queue_);

  laser_sub_ = nh_.subscribe(ops);
  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(name + "/scan_data", 10);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>(name + "/cloud_data", 10);
}

void Lidar::setScanCount(int scan_count){
  scan_count_ = scan_count;
}

int Lidar::getScanCount() {
  return scan_count_;
}

void Lidar::sendData()
{
  queue_.callAvailable();
}

void Lidar::callback_(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // bag_.write("scan_" + topic_suffix_, ros::Time::now(), scan);
  scan_pub_.publish(scan);

  sensor_msgs::PointCloud cloud;
  projection_.transformLaserScanToPointCloud("laser", *scan, cloud, tf_);
  // bag_.write("cloud_" + topic_suffix_, ros::Time::now(), cloud);
  cloud_pub_.publish(cloud);

  ROS_INFO("Scan no: %d", scan_number_);
  scan_number_ = ++scan_number_ % scan_count_;
}
