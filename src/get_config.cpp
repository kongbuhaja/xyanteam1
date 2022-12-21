#include "xyanteam1/get_config.h"

namespace xycar {
void getConfig() {
  std::string config_path;
  nh_.getParam("config_path", config_path);
  // std::cout << config_path << std::endl;
  YAML::Node config = YAML::LoadFile(config_path);
  // std::cout << config << std::endl;
  setXycarParams(config);
  setLidarParams(config);
}

void GetConfig::setXycarParams(const YAML::Node &config) {
  xycar_speed_ = config["XYCAR"]["START_SPEED"].as<float>();
  xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
  xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
  xycar_speed_control_threshold_ = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
  acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
  deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
}

void GetConfig::setLidarParams(const YAML::Node &config) {
  LIDAR_START_DEGREE = config["LIDAR"]["LIDAR_START_DEGREE"].as<int>();
  LIDAR_END_DEGREE = config["LIDAR"]["LIDAR_END_DEGREE"].as<int>();
  LIDAR_MAX_DEGREE = config["LIDAR"]["LIDAR_MAX_DEGREE"].as<int>();  
  LIDAR_OBJECT_MIN_SIZE = config["LIDAR"]["LIDAR_OBJECT_MIN_SIZE"].as<int>();
  LIDAR_MINIMUM_DISTANCE = config["LIDAR"]["LIDAR_MINIMUM_DISTANCE"].as<float>();
  LIDAR_MAXIMUM_DISTANCE = config["LIDAR"]["LIDAR_MAXIMUM_DISTANCE"].as<float>();
  STEERING_TOTAL_NUMBER = config["LIDAR"]["STEERING_TOTAL_NUMBER"].as<float>();
}
} // namespace xycar