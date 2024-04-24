#include "station2/StationClient.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "station_client");
  ros::NodeHandle nh;

  StationClient client(nh, "rplidar", "rplidar_corrected", "camera_astra");

  while (ros::ok()) {
    int key = client.getch();

    if (key == KEYCODE_S) client.bagOpen();
    else if (key == KEYCODE_P) client.bagClose();

    client.sendKey(key);

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}