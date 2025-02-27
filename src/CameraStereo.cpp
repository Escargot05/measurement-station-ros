#include "measurement_station/CameraStereo.h"

CameraStereo::CameraStereo(ros::NodeHandle& nh, std::string name) : Camera(nh, name)
{
  std::string ir2_topic;

  nh.getParam("station/" + name + "/ir2_topic", ir2_topic);

  ros::SubscribeOptions ops5 = ros::SubscribeOptions::create<sensor_msgs::Image>(
    ir2_topic, 1, boost::bind(&CameraStereo::ir2Callback_, this, _1), ros::VoidPtr(), &queue_);

  ir2_sub_ = nh_.subscribe(ops5);

  ir2_pub_ = nh_.advertise<sensor_msgs::Image>(name + "/ir2_data", 10);
}


void CameraStereo::ir2Callback_(const sensor_msgs::Image::ConstPtr& img)
{
  ir2_pub_.publish(img);
}
