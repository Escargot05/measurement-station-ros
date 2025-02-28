#include "measurement_station/StationClient.h"

StationClient::StationClient(ros::NodeHandle& nh, std::string lidar_name, std::string camera_name,
                             std::string camera2_name)
  : nh_(nh)
{
  key_code_ = nh_.advertise<std_msgs::Int32>("station/command", 1);

  std::array<std::string, 11> topics = { 
    lidar_name + "/scan_data",    
    lidar_name + "/cloud_data",
    camera_name + "/cloud_data",  
    camera_name + "/color_data",
    camera_name + "/depth_data",  
    camera_name + "/ir_data",
    camera2_name + "/cloud_data", 
    camera2_name + "/color_data",
    camera2_name + "/depth_data", 
    camera2_name + "/ir_data",
    camera2_name + "/ir2_data" 
  };

  for (const auto& topic : topics)
  {
    if (!topic.compare(lidar_name + "/cloud_data"))
    {
      subscribers_.emplace_back(nh_.subscribe<sensor_msgs::PointCloud>(
          topic, 1, boost::bind(&StationClient::callback_<sensor_msgs::PointCloud>, this, _1, topic)));

    }
    else if (topic.find("cloud_data") != std::string::npos)
    {
      subscribers_.emplace_back(nh_.subscribe<sensor_msgs::PointCloud2>(
          topic, 1, boost::bind(&StationClient::callback_<sensor_msgs::PointCloud2>, this, _1, topic)));
    }
    else if (topic.find("scan_data") != std::string::npos)
    {
      subscribers_.emplace_back(nh_.subscribe<sensor_msgs::LaserScan>(
          topic, 1, boost::bind(&StationClient::callback_<sensor_msgs::LaserScan>, this, _1, topic)));
    }
    else
    {
      subscribers_.emplace_back(nh_.subscribe<sensor_msgs::Image>(
          topic, 1, boost::bind(&StationClient::callback_<sensor_msgs::Image>, this, _1, topic)));
    }
  }

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

template <typename T>
void StationClient::callback_(const typename T::ConstPtr& msg, const std::string& topic)
{
  bag_.write(topic, ros::Time::now(), msg);
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
