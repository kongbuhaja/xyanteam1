#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "alcoholdriving/motor.h"
#include "alcoholdriving/vision.h"
#include "alcoholdriving/pid_controller.h"

#define MIN_ERROR_IGNORABLE -1
#define MAX_ERROR_IGNORABLE 1
#define MAIN_DEBUG true
class Main
{
public:
    Main(int argc, char **argv)
        : argc_(argc), argv_(argv)
    {

        ros::init(argc_, argv_, "team1/main");
        nh_ = ros::NodeHandle();

        /* NOTE : Make it sure that `config_path` is valid in .launch params. */
        const YAML::Node &config = YAML::LoadFile(nh_.getParam("config_path", config_path));
        xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
        xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
        xycar_speed_control_threshold_ = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
        acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
        deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
        motor_ptr_ = new alcoholdriving::Motor(nh_, config["XYCAR"]["START_SPEED"].as<float>());
        pid_ptr_ = new alcoholdriving::PID(config["PID"]["P_GAIN"].as<float>(),
                                           config["PID"]["I_GAIN"].as<float>(),
                                           config["PID"]["D_GAIN"].as<float>());
        line_detector_ptr_ = new alcoholdriving::LineDetector(nh_);
    }
    ~Main()
    {
        // TODO : Destroy
        delete motor_ptr_;
        delete pid_ptr_;
        delete line_detector_ptr_;
    }

    int run()
    {

        while (ros::ok())
        {
            // Vision proccessing
            error_ = line_detector_ptr_-> getError();
            
            // TODO : LiDAR

            // TOBE : IMU

            // float angle = std::max(-(float)kXycarSteeringAngleLimit,
            //                        std::min(pid_ptr_->getControlOutput(error, ~(error_ > MIN_ERROR_IGNORABLE && error_ < MAX_ERROR_IGNORABLE)),
            //                                 (float)kXycarSteeringAngleLimit));
            float angle = std::max(-(float)kXycarSteeringAngleLimit,
                                   std::min(pid_ptr_->getControlOutput(error_),
                                            (float)kXycarSteeringAngleLimit));
            motor_.set_motor_control(
                /* angle */ angle,
                /* speed */ speed_control(angle));

            // Motor and steering control
            motor_.motor_publish();
        }

        return 0;
    }

private:
    static const int kXycarSteeringAngleLimit = 50;

    int argc_;
    char **argv_;
    bool debug_;
    float error_;     

    // Xycar Device variables
    float xycar_max_speed_;
    float xycar_min_speed_;
    float xycar_speed_control_threshold_;
    float acceleration_step_;
    float deceleration_step_;
    float xycar_initial_speed_;

    ros::NodeHandle nh_;
    std::unique_ptr<alcoholdriving::Motor> motor_ptr_;
    std::unique_ptr<alcoholdriving::PID> pid_ptr_;
    std::unique_ptr<alcoholdriving::LineDetector> line_detector_ptr_;

    inline int speed_control(float angle)
    {
        // decelerate if the angle is wide enough,
        // else accelerate.
        return std::abs(angle) > xycar_speed_control_threshold_ ? std::max(motor_.getSpeed() - deceleration_step_, xycar_min_speed_) : std::min(motor_.getSpeed() + acceleration_step_, xycar_max_speed_);
    }
};

int main(int argc, char **argv)
{
    return Main(argc, argv).run();
}
