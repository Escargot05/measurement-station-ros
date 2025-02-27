#ifndef CAMERASTEREO_H
#define CAMERASTEREO_H

#include "measurement_station/Camera.h"

class CameraStereo : public Camera
{
public:
  CameraStereo(ros::NodeHandle& nh, std::string name);

private:
  ros::Subscriber ir2_sub_;
  ros::Publisher ir2_pub_;

  void ir2Callback_(const sensor_msgs::Image::ConstPtr& img);
};

#endif