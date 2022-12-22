#ifndef LIDAR_OBJECT_DETECTION_H_
#define LIDAR_OBJECT_DETECTION_H_
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <algorithm>

namespace alcoholdriving
{
    class ObstacleLidarDetector
    {
    public:
        ObstacleLidarDetector(const ros::NodeHandle nh_);
        ~ObstacleLidarDetector();
        void run();
        float& getResultArray();

    private:
        ros::NodeHandle nh;
        ros::Subscriber lidar_sub;
        std::vector<float> lidar_points;
        float* result_array;

        void lidar_callback(sensor_msgs::LaserScan data);
        bool is_lidar_on();
        bool detect_object_lidar();
    };
}
#endif

