#include <StationController.h>

// In case of wondering what used gcode commands do: https://marlinfw.org/meta/gcode/
// The sent position is relative to the current postion
// The sleep times were determined experimentally and are dependent on feedrates

void StationController::printPlannerPosition_()
{
  serial_.write("M114\r\n");
  ROS_INFO_STREAM(serial_.read(serial_.available()));
}

StationController::StationController(ros::NodeHandle& nh) : nh_(nh)
{
  nh_.getParam("station/serial/port", port_);
  nh_.getParam("station/serial/baudrate", baudrate_);
  nh_.getParam("station/offset", offset_);

  nh_.getParam("station/initial_position", distance_.start);
  nh_.getParam("station/initial_angle", angle_.start);
  nh_.getParam("station/final_position", distance_.stop);
  nh_.getParam("station/final_angle", angle_.stop);
  nh_.getParam("station/angular_step", angle_.step);
  nh_.getParam("station/linear_step", distance_.step);

  distance_pub_ = nh_.advertise<std_msgs::Int32>("station/distance", 2);
  angle_pub_ = nh_.advertise<std_msgs::Int32>("station/angle", 2);
}

void StationController::init()
{
  ROS_INFO("Port: %s", port_.c_str());

  serial_.setPort(port_);
  serial_.setBaudrate(baudrate_);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(to);
  serial_.open();
  ros::Duration(5.0).sleep();

  ROS_INFO_STREAM("Port has been open sucessfully");
  ROS_INFO_STREAM(serial_.read(serial_.available()));

  // Enabling cold extrusion
  // Marker is mounted on extruder axis, so this protection have to be disabled
  // Maybe it should be done on Marlin level
  serial_.write("M302 S0\n");
  ROS_INFO_STREAM(serial_.read(serial_.available()));
}

void StationController::home()
{
  serial_.write("G28 X\r\n");
  ROS_INFO("Homing X axis");
  ROS_INFO_STREAM(serial_.read(serial_.available()));
  ros::Duration(140.0).sleep();
}

void StationController::moveLinearStep()
{
  distance_.current -= distance_.step;
  std::string gcode = "G0 X" + std::to_string(distance_.current) + "F6000\r\n";
  serial_.write(gcode);

  ROS_INFO("Move to distance: %d", distance_.current);
  ros::Duration(1.0).sleep();
}

void StationController::moveAngularStep()
{
  angle_.current += angle_.step;
  std::string gcode = "G1 E" + std::to_string(angle_.current) + " F1000\r\n";
  serial_.write(gcode);

  ROS_INFO("Move to angle: %d", angle_.current);
  ros::Duration(1.0).sleep();
}

void StationController::moveLinearContinous()
{
  distance_.current = distance_.stop;
  std::string gcode = "G0 X" + std::to_string(distance_.current) + " F6000\r\n";
  serial_.write(gcode);

  ROS_INFO("Move to distance: %d", distance_.current);
}

void StationController::moveDistanceMax()
{
  distance_.current = distance_.start;
  std::string gcode = "G0 X" + std::to_string(distance_.current) + " F6000\r\n";
  serial_.write(gcode);

  printPlannerPosition_();
  ROS_INFO("Move to distance: %d", distance_.current);
  ros::Duration(30.0).sleep();
}

void StationController::moveAngleStart()
{
  std::string gcode = "G1 E" + std::to_string(angle_.start) + " F1000\r\n";
  serial_.write(gcode);

  printPlannerPosition_();
  ROS_INFO("Marker at initial angle: %d", angle_.current);
  ros::Duration(1.0).sleep();
}

bool StationController::LinearStop() {
  return (distance_.current == distance_.stop ? true : false);
}

bool StationController::AngularStop() {
  return (angle_.current == angle_.stop ? true : false);
}

void StationController::enableSteppers()
{
  serial_.write("M18 X E\r\n");
  ROS_INFO_STREAM(serial_.read(serial_.available()));
  ROS_INFO("Steppers enabled: Carriage and marker can be moved freely");
}

void StationController::disableSteppers()
{
  serial_.write("M17 X E\r\n");
  ROS_INFO_STREAM(serial_.read(serial_.available()));
  ROS_INFO("Steppers disabled: Do not move maraker and carriage");
}

void StationController::incrementAngle()
{
  std::string gcode = "G1 E1 F1000\r\n";
  serial_.write(gcode);
}

void StationController::decrementAngle()
{
  std::string gcode = "G1 E-1 F1000\r\n";
  serial_.write(gcode);
}

void StationController::sendDistanceData()
{
  std_msgs::Int32 distance;
  distance.data = distance_.current + offset_;
  distance_pub_.publish(distance);
  // bag_.write("distance", ros::Time::now(), distance);
  // bag_.write("angle", ros::Time::now(), angle);
}

void StationController::sendAngleData()
{
  std_msgs::Int32 angle;
  angle.data = angle_.current;
  angle_pub_.publish(angle);
}
