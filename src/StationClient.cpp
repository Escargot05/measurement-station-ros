#include "station2/StationClient.h"

StationClient::StationClient(ros::NodeHandle& nh, std::string lidar_name, std::string lidar2_name,
                             std::string camera_name)
  : nh_(nh)
{
  key_code_ = nh_.advertise<std_msgs::Int32>("station/command", 1);

  laser_ = nh_.subscribe<sensor_msgs::LaserScan>(lidar_name + "/scan_data", 10, &StationClient::laserCallback_, this);
  cloud_ = nh_.subscribe<sensor_msgs::PointCloud>(lidar_name + "/cloud_data", 10, &StationClient::cloudCallback_, this);
  laser_corrected_ = nh_.subscribe<sensor_msgs::LaserScan>(lidar2_name + "/scan_data", 10, &StationClient::laserCorrectedCallback_, this);
  cloud_corrected_ = nh_.subscribe<sensor_msgs::PointCloud>(lidar_name + "/cloud_data", 10, &StationClient::cloudCorrectedCallback_, this);
  camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_name + "/info_data", 10, &StationClient::cameraInfoCallback_, this);
  camera_color = nh_.subscribe<sensor_msgs::Image>(camera_name + "/color_data", 10, &StationClient::cameraColorCallback_, this);
  camera_depth_ = nh_.subscribe<sensor_msgs::Image>(camera_name + "/depth_data", 10, &StationClient::cameraDepthCallback_, this);
  camera_ir_ = nh_.subscribe<sensor_msgs::Image>(camera_name + "/ir_data", 10, &StationClient::cameraIrCallback_, this);
  distance_ = nh_.subscribe<std_msgs::Int32>("/station/distance", 10, &StationClient::distanceCallback_, this);
  angle_ = nh_.subscribe<std_msgs::Int32>("station/angle", 10, &StationClient::angleCallback_, this);

  ROS_INFO("Client started!");
}

// Custom getchar method to skip the input buffer
// It uses OS specific hack - Linux in this case
int StationClient::getch()
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

void StationClient::bagOpen()
{
  std::string bag_name;
  std::string bag_location;

  nh_.getParam("/station/bag_name", bag_name);
  nh_.getParam("/station/bag_location", bag_location);

  bag_.open(bag_location + "/" + bag_name + ".bag", rosbag::bagmode::Write);
  ROS_INFO("Bag opened");
}

void StationClient::bagClose()
{
  bag_.close();
  ROS_INFO("Bag closed");
}

void StationClient::laserCallback_(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  bag_.write("laser", ros::Time::now(), scan);
  ROS_INFO("Laser stored.");
}

void StationClient::laserCorrectedCallback_(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  bag_.write("laser_corrected", ros::Time::now(), scan);
}

void StationClient::cloudCallback_(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  bag_.write("cloud", ros::Time::now(), cloud);
}

void StationClient::cloudCorrectedCallback_(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  bag_.write("cloud_corrected", ros::Time::now(), cloud);
}

void StationClient::cameraInfoCallback_(const sensor_msgs::CameraInfo::ConstPtr& info)
{
  bag_.write("camera_info", ros::Time::now(), info);
}

void StationClient::cameraColorCallback_(const sensor_msgs::Image::ConstPtr& img)
{
  bag_.write("color_img", ros::Time::now(), img);
  ROS_INFO("Image stored.");
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
}

void StationClient::angleCallback_(const std_msgs::Int32::ConstPtr& num)
{
  bag_.write("angle", ros::Time::now(), num);
}

void StationClient::sendKey(int c)
{
  std_msgs::Int32 num;
  num.data = c;
  key_code_.publish(num);

  ROS_INFO("Key %d sended", c);
}
