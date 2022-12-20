#include "ros/ros.h"
// #include "lane/xycar_motor.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
#include "algorithm"
#include "std_msgs/Float32.h"

namespace team1
{
// #define USE_MOTOR

/* Start degree of Lidar(Left handed) */
#define LIDAR_START_DEGREE 350

/* End degree of Lidar(Right handed) */
#define LIDAR_END_DEGREE 110

/* Maxinum degree of Lidar (Mininmum is 0)*/
#define LIDAR_MAX_DEGREE 505

/* Required minimum number of dots for an object */
#define LIDAR_MIN_SIZE 20

typedef struct team1_lidar {
    float distance;
    float angle;
} Team1_Lidar; 

}
class CAR
{
private:
    
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
#ifdef USE_MOTOR
    ros::Publisher motor_pub;
    ros::Subscriber motor_sub;
    lane::xycar_motor msg_motor;
#endif
    std::vector<float> lidar_points;
    std::vector<float> lidar_intensities;

    void lidar_callback(sensor_msgs::LaserScan data)
    {
        lidar_points = data.ranges;

    }


    bool is_lidar_on()
    {
        return (lidar_points.size() != 0);
    }

    bool detect_object_lidar(float threshold = 0.3)
    {
        // return false;
        // int start = team1::LIDAR_START_DEGREE;
        // int end = team1::LIDAR_END_DEGREE;
        int start = LIDAR_START_DEGREE;
        int end = LIDAR_END_DEGREE;
        int length = lidar_points.size();
        int front_size = length - start + end + 2;
        std::vector<float> front;
        int count = 20;

#if (0)
        for (int i = start; i < lidar_points.size(); i++)
        {
            front.push_back(lidar_points[i]);
        }
        for (int i = 0; i < end; i++)
        {
            front.push_back(lidar_points[i]);
        }

        for (int i = 0; i < front.size(); i++)
        {
            if (front[i] <= threshold)
            {
                count -= 1;
                if (count <= 0)
                {
                    ROS_ERROR("lidar : [%d]", (start + i) % LIDAR_MAX_DEGREE);
                    return true;
                }
            }
            else
            {
                count = 20;
            }
        }

#else
        for (int i = 0; i < lidar_points.size(); i++)
        {
            // if ((lidar_points[i] >= 0.3f) && (lidar_points[i] <= 1.0f))
            if (lidar_points[i] >= 0.3f)
            {
                ROS_ERROR("lidar_points[%d][%f]", i, lidar_points[i]);
            }
        }
#endif
        memset(&lidar_points, 0x0, sizeof(lidar_points));
        return false;
    }

public:
    CAR(const ros::NodeHandle nh_) : nh(nh_)
    {
        lidar_sub = nh.subscribe("/scan", 1, &CAR::lidar_callback, this);
#ifdef USE_MOTOR
        motor_pub = nh.advertise<lane::xycar_motor>("xycar_motor", 1);
        motor_sub = nh.subscribe("go", &CAR::motor_callback, this);
#endif
    }

    void run()
    {
        if (is_lidar_on())
        {
            if (detect_object_lidar())
            {
            }
            else
            {
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_driver");
    ros::NodeHandle nh_;
    CAR xycar(nh_);
    ros::Rate rate(30);
    ROS_ERROR("Start Lidar");
    while (ros::ok())
    {
        xycar.run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}