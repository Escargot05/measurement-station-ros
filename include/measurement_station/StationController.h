#ifndef STATIONCONTROLLER_H_
#define STATIONCONTROLLER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <serial/serial.h>

struct position
{
  int start;
  int stop;
  int step;
  int current;
};

class StationController
{

public:
  StationController(ros::NodeHandle& nh);

  void init();
  void home();
  void moveLinearStep();
  void moveAngularStep();
  void moveLinearContinous();
  void moveDistanceMax();
  void moveAngleStart();
  bool LinearStop();
  bool AngularStop();

  void enableSteppers();
  void disableSteppers();
  void incrementAngle();
  void decrementAngle();

  void sendDistanceData();
  void sendAngleData();

private:
  ros::NodeHandle& nh_;

  ros::Publisher distance_pub_;
  ros::Publisher angle_pub_;

  serial::Serial serial_;

  std::string port_;
  int baudrate_;

  position distance_;
  position angle_;

  int offset_;

  void printPlannerPosition_();
};

#endif
