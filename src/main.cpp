#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "alcoholdriving/motor.h"

#define MIN_ERROR_IGNORABLE -1
#define MAX_ERROR_IGNORABLE 1
#define MAIN_DEBUG true

class Main
{
public:
    Main(int argc, char **argv, bool debug = false)
        : argc_(argc), argv_(argv), debug_(debug)
    {
        ros::init(argc_, argv_, "team1/main");
        nh_ = ros::NodeHandle();
        motor_ptr_ = new Motor();
    }
    ~Main()
    {
        // TODO : Destroy
    }

    int run()
    {

        while (ros::ok())
        {
            // TODO : Vision

            // TODO : LiDAR

            // TOBE : IMU

            // MAX SPEED when error is zero enough. (PD)
            if (error_ > MIN_ERROR_IGNORABLE && error_ < MAX_ERROR_IGNORABLE)
            {
                /* TODO */
                motor_.set_motor_control(
                    /* angle */ 0,
                    /* speed */ 0);
            }

            // LOWER SPEED when it needs to change the angle. (PID)
            else
            {
                /* TODO */
                motor_.set_motor_control(
                    /* angle */ 0,
                    /* speed */ 0);
            }

            // Motor and steering control
            motor_.motor_publish();
        }

        return 0;
    }
    

private:
    int argc_;
    char **argv_;
    bool debug_;
    float error_;

    ros::NodeHandle nh_;
    std::unique_ptr<Motor> motor_ptr_;

};

int main(int argc, char **argv)
{
    return Main(argc, argv, /*debug=*/MAIN_DEBUG_MODE).run();
}
