#include "station2/StationServer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "station_server");
  ros::NodeHandle nh("~");

  StationServer server(nh, "lidar", "lidar2", "camera");

  ros::spin();

  return 0;
}