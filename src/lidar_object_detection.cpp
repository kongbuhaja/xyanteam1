#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
#include "algorithm"
#include "std_msgs/Float32.h"

namespace team1
{
/* Start degree of Lidar(Left handed) */
#define LIDAR_START_DEGREE 350

/* End degree of Lidar(Right handed) */
#define LIDAR_END_DEGREE 110

/* Maxinum degree of Lidar (Mininmum is 0)*/
#define LIDAR_MAX_DEGREE 505

/* Required minimum number of dots for an object */
#define LIDAR_OBJECT_MIN_SIZE 6

/* Required minimum distance of meters of an object */
#define LIDAR_MINIMUM_DISTANCE 0.3f

/* Required maximum distance of meters of an object */
#define LIDAR_MAXIMUM_DISTANCE 1.f

#define STEERING_TOTAL_NUMBER 40
}

using namespace team1;

class ObstacleLidarDetector
{
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    std::vector<float> lidar_points;
    float* result_array;

    /* Callback triggered if topic sent from Lidar */
    void lidar_callback(sensor_msgs::LaserScan data)
    {
        lidar_points = data.ranges;
    }
    bool is_lidar_on()
    {
        return lidar_points.size() != 0;
    }
    bool detect_object_lidar()
    {
        /*
            data.ranges
                index : angle
                element : distance
        */
        /* counting cloud points in ranges */
        int cnt = LIDAR_START_DEGREE - LIDAR_END_DEGREE;
        float x[LIDAR_START_DEGREE - LIDAR_END_DEGREE] = {0,};
        int number_of_adjacent = 0;
        int j;

        /* [!] Source code verification required [!] */
        for (int i = 0; i < cnt; i++)
        {
            j = (i + LIDAR_START_DEGREE) % LIDAR_MAX_DEGREE;
            if ((lidar_points[j] > LIDAR_MINIMUM_DISTANCE) && (lidar_points[j] < LIDAR_MAXIMUM_DISTANCE))
            {
                x[i] = lidar_points[j];
            }
        }

        float distance_avg;
        for (int i = 0, k = 0; (i+(LIDAR_OBJECT_MIN_SIZE-1)) < cnt; i += LIDAR_OBJECT_MIN_SIZE, k++)
        {
            if (x[i] && x[i+1] && x[i+2] && x[i+3] && x[i+4] && x[i+5])
            {
                distance_avg = x[i];
                distance_avg += x[i+1];
                distance_avg += x[i+2];
                distance_avg += x[i+3];
                distance_avg += x[i+4];
                distance_avg += x[i+5];
                distance_avg = distance_avg / (float)LIDAR_OBJECT_MIN_SIZE;

                result_array[k] = distance_avg;
                ++number_of_adjacent;
            }
            else
            {
                result_array[k] = 0.0f;
            }            
        }
        return number_of_adjacent != 0;
    }

public:
    ObstacleLidarDetector(const ros::NodeHandle nh_) : nh(nh_)
    {
        result_array = new float[STEERING_TOTAL_NUMBER];
        lidar_sub = nh.subscribe("/scan", 1, &ObstacleLidarDetector::lidar_callback, this);
    }
    ~ObstacleLidarDetector(){
        delete result_array;
    }
    void run()
    {
        if (is_lidar_on())
        {
            if (detect_object_lidar())
            {
                    /* ToDo */
            }
            else
            {
            }
        }
    }

    
    // TODO 
    float& getResultArray(){
        return *result_array;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_driver");
    ros::NodeHandle nh_;
    ROS_WARN("Start Lidar");
    ObstacleLidarDetector xycar(nh_);
    ros::Rate rate(30);

    while (ros::ok())
    {
        xycar.run();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
