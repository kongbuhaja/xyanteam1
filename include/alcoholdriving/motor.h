#ifndef MOTOR_H_
#define MOTOR_H_

#include "ros/ros.h"
#include "lane/xycar_motor.h"

namespace alcoholdriving
{
    class Motor
    {
    public:
        alcoholdriving::Motor()
        {
            motor_pub_ = nh_.advertise<lane::xycar_motor>("xycar_motor", 1);
            motor_sub_ = nh_.subscribe("go", &Main::motor_callback, this);
        }

        alcoholdriving::~Motor() {}

        inline void alcoholdriving::motor_callback(const lane::xycar_motor &msg)
        {
            motor_publish(msg);
        }
        inline void alcoholdriving::motor_publish()
        {
            motor_pub.publish(msg_motor_);
        }
        inline void alcoholdriving::set_motor_control(const float angle, const float speed)
        {
            msg_motor_.angle = angle;
            msg_motor_.speed = speed;
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher motor_pub_;
        ros::Subscriber motor_sub_;
        lane::xycar_motor msg_motor_;
    };
}
#endif