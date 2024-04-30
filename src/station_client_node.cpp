#include "station2/StationClient.h"
#include <thread>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "station_client");
  ros::NodeHandle nh;

  StationClient client(nh, "rplidar", "rplidar_corrected", "camera_astra");

  std::thread keyInput(&StationClient::getInput, &client);
  keyInput.detach();

  ros::spin();

  return 0;
}