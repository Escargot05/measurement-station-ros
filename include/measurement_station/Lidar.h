#ifndef LIDAR_H
#define LIDAR_H

#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class Lidar
{
public:
  Lidar(ros::NodeHandle& nh, std::string name);

  static void setScanCount(int scan_count);
  static int getScanCount();
  void sendData();

private:
  ros::NodeHandle& nh_;
  tf::TransformListener tf_;
  ros::CallbackQueue queue_;
  laser_geometry::LaserProjection projection_;

  ros::Subscriber laser_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher cloud_pub_;

  static int scan_count_;
  int scan_number_;

  void callback_(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif
