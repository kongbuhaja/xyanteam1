#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <xycar_msgs/xycar_motor.h>
#include <yaml-cpp/yaml.h>

//#include "lane_keeping_system/hough_line.h"
#include "lane_keeping_system/pid_controller.h"

namespace xycar {
class GetConfig {
public:
  // Set parameters from config file
  void setXycarParams(const YAML::Node &config);

  // Set LIDAR from config file
  void setLidarParams(const YAML::Node &config);

private:
  // Xycar Device variables
  float xycar_speed_;
  float xycar_max_speed_;
  float xycar_min_speed_;
  float xycar_speed_control_threshold_;
  float acceleration_step_;
  float deceleration_step_;

  // LIDAR variables
  float LIDAR_START_DEGREE;
  float LIDAR_END_DEGREE;
  float LIDAR_MAX_DEGREE;
  float LIDAR_OBJECT_MIN_SIZE;
  float LIDAR_MINIMUM_DISTANCE;
  float LIDAR_MAXIMUM_DISTANCE;
  float STEERING_TOTAL_NUMBER;

  // Xycar Steering Angle Limit
  static const int kXycarSteeringAngleLimit = 50;
  // PID Class for Control
  PID *pid_ptr_;
};
}  // namespace xycar

