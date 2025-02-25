#include "measurement_station/StationClient.h"

StationClient::StationClient(ros::NodeHandle& nh, std::string lidar_name, std::string camera_name)
  : nh_(nh)
{
  key_code_ = nh_.advertise<std_msgs::Int32>("station/command", 1);

  laser_ = nh_.subscribe(lidar_name + "/scan_data", 1, &StationClient::laserCallback_, this);
  cloud_ = nh_.subscribe(lidar_name + "/cloud_data", 1, &StationClient::cloudCallback_, this);
  camera_info_ = nh_.subscribe(camera_name + "/info_data", 1, &StationClient::cameraInfoCallback_, this);
  camera_color = nh_.subscribe(camera_name + "/color_data", 1, &StationClient::cameraColorCallback_, this);
  camera_depth_ = nh_.subscribe(camera_name + "/depth_data", 1, &StationClient::cameraDepthCallback_, this);
  camera_ir_ = nh_.subscribe(camera_name + "/ir_data", 1, &StationClient::cameraIrCallback_, this);
  distance_ = nh_.subscribe("station/distance", 10, &StationClient::distanceCallback_, this);
  angle_ = nh_.subscribe("station/angle", 10, &StationClient::angleCallback_, this);

  ros::Duration(3.0).sleep();
  ROS_INFO("Client started!");
}

void StationClient::getInput()
{
  while (ros::ok())
  {
    int key = getch_();

    if (key == KEYCODE_S)
      bagOpen_();
    else if (key == KEYCODE_P)
      bagClose_();

    sendKey_(key);
  }
}

// Custom getchar method to skip the input buffer
// It uses OS specific hack - Linux in this case
int StationClient::getch_()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);

  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  int c = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return c;
}

void StationClient::bagOpen_()
{
  std::string bag_name;
  std::string bag_location;

  nh_.getParam("/station/bag_name", bag_name);
  nh_.getParam("/station/bag_location", bag_location);

  bag_.open(bag_location + "/" + bag_name + ".bag", rosbag::bagmode::Write);
  ROS_INFO("Bag opened");
}

void StationClient::bagClose_()
{
  bag_.close();
  ROS_INFO("Bag closed");
}

void StationClient::sendKey_(int c)
{
  std_msgs::Int32 num;
  num.data = c;
  key_code_.publish(num);

  ROS_INFO("Key %d sended", c);
}

void StationClient::laserCallback_(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  bag_.write("laser", ros::Time::now(), scan);
}

void StationClient::cloudCallback_(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  bag_.write("cloud", ros::Time::now(), cloud);
}

void StationClient::cameraInfoCallback_(const sensor_msgs::CameraInfo::ConstPtr& info)
{
  bag_.write("camera_info", ros::Time::now(), info);
}

void StationClient::cameraColorCallback_(const sensor_msgs::Image::ConstPtr& img)
{
  bag_.write("color_img", ros::Time::now(), img);
}

void StationClient::cameraDepthCallback_(const sensor_msgs::Image::ConstPtr& img)
{
  bag_.write("depth_img", ros::Time::now(), img);
}

void StationClient::cameraIrCallback_(const sensor_msgs::Image::ConstPtr& img)
{
  bag_.write("ir_img", ros::Time::now(), img);
}

void StationClient::distanceCallback_(const std_msgs::Int32::ConstPtr& num)
{
  bag_.write("distance", ros::Time::now(), num);
  ROS_INFO("Distance stored");
}

void StationClient::angleCallback_(const std_msgs::Int32::ConstPtr& num)
{
  bag_.write("angle", ros::Time::now(), num);
  ROS_INFO("Angle stored");
}

