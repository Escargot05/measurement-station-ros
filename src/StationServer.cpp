#include "station2/StationServer.h"

StationServer::StationServer(ros::NodeHandle& nh, std::string lidar_name, std::string lidar2_name,
                             std::string camera_name)
  : nh_(nh)
  , rplidar_(nh, lidar_name)
  , rplidar_corrected_(nh, lidar2_name)
  , astra_(nh, camera_name)
  , station_(nh)
  , MEASUREMENT_TIME_(30.0)
{
  int image_count;
  int scan_count;

  key_code_ = nh_.subscribe("station/command", 1, &StationServer::keyCallaback_, this);
  station_.init();

  nh_.getParam("station/continous_measurement", continous_measurement_);
  nh_.getParam("station/camera_images", image_count);
  nh_.getParam("station/lidar_scans", scan_count);
  Camera::setImageCount(image_count);
  Lidar::setScanCount(scan_count);

  ROS_INFO("Server started!");
}

void StationServer::keyCallaback_(const std_msgs::Int32::ConstPtr& num)
{
  key_ = num->data;
  ROS_INFO("Key %d received", key_);

  if (key_ == KEYCODE_P)
    ros::shutdown();
}

void StationServer::harvestData_()
{
  ROS_INFO("Starting data harvesting");

  station_.home();
  station_.moveAngleStart();
  station_.moveDistanceMax();

  if (continous_measurement_)
  {
    station_.moveLinearContinous();

    double measurement_time = ros::Time::now().toSec() + MEASUREMENT_TIME_;
    while (ros::Time::now().toSec() < measurement_time)
    {
      // ros::spinOnce();
      rplidar_.sendData();
      rplidar_corrected_.sendData();
      astra_.sendData();
      ros::Duration(0.1).sleep();
    }
  }
  else
  {
    // There should be versions which depends on following images and scans values.
    // For now the only one case is implemented

    int images = Camera::getImageCount();
    int scans = Lidar::getScanCount();

    assert(images <= scans);
    int rate = scans / images;

    while (true)
    {
      station_.sendAngleData();

      while (true)
      {
        station_.sendDistanceData();

        for (int i = 0; i < scans; i++)
        {
          if (!(i % rate))
            astra_.sendData();
          rplidar_.sendData();
          rplidar_corrected_.sendData();

          ros::Duration(0.1).sleep();
        }
        if (station_.LinearStop())
          break;
        station_.moveLinearStep();
      }

      if (station_.AngularStop())
        break;
      station_.moveDistanceMax();
      station_.moveAngularStep();
    }

    ROS_INFO("Data harvested succesfully");
  }
}

void StationServer::performAction()
{
  switch (key_)
  {
    case KEYCODE_E:
      station_.enableSteppers();
      break;

    case KEYCODE_D:
      station_.disableSteppers();
      break;

    case KEYCODE_H:
      station_.incrementAngle();
      break;

    case KEYCODE_L:
      station_.decrementAngle();
      break;

    case KEYCODE_S:
      this->harvestData_();
      break;
  }

  key_ = 0;
}
