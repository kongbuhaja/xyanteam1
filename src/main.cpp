#include <cmath>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "lane/xycar_motor.h"

#define MIN_ERROR_IGNORABLE -1
#define MAX_ERROR_IGNORABLE 1

class Main
{
private:
    int argc_;
    char **argv_;
    bool debug_;
    float error_;

    lane::xycar_motor msg_motor_;
    ros::NodeHandle nh_;
    ros::Publisher motor_pub_;
    ros::Subscriber motor_sub_;

public:
    Main(int argc, char **argv, bool debug = false) : argc_(argc), argv_(argv), debug_(debug)
    {
        ros::init(argc_, argv_, "team1/main");

        motor_pub = nh.advertise<lane::xycar_motor>("xycar_motor", 1);
        motor_sub = nh.subscribe("go", &Main::motor_callback, this);

        nh_ = ros::NodeHandle();
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
                set_motor_control(
                    /* angle */ 0,
                    /* speed */ 0);
            }

            // LOWER SPEED when it needs to change the angle. (PID)
            else
            {
                /* TODO */
                set_motor_control(
                    /* angle */ 0,
                    /* speed */ 0);
            }

            // Motor and steering control
            motor_pub.publish(msg_motor_);
        }

        return 0;
    }
    inline void motor_callback(const lane::xycar_motor &msg)
    {
        motor_publish(msg);
    }
    inline void motor_publish()
    {
        motor_pub.publish(msg_motor_);
    }
    inline void set_motor_control(const float angle, const float speed)
    {
        msg_motor_.angle = angle;
        msg_motor_.speed = speed;
    }
};

int main(int argc, char **argv)
{
    return Main(argc, argv, true).run();
}
