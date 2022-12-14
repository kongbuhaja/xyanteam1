#ifndef VISION_H_
#define VISION_H_

#include "ros/ros.h"
#include "iostream"
#include "random"
#include "opencv2/opencv.hpp"

#include <cmath>
#include <numeric>

namespace alcoholdriving
{
#define PI M_PI
#define LOW_THRESHOLD 60
#define HIGH_THRESHOLD 70
#define SLOPE_THRESHOLD 10
#define OFFSET 380
#define GAP 20
#define WIDTH 640
#define HEIGHT 480
#define RHO 1
#define THETA PI / 180
#define LIMIT_SLOPE 10
#define IMAGES_SIZE 30
#define MA_SIZE 50

    class LineDetector
    {
    public:
        LineDetector(ros::NodeHandle nh);
        ~LineDetector();

    private:
        cv::Mat image;
        alcoholdriving::MovingAverage ma;
        ros::NodeHandle nh_;
        std::pair<std::vector<float>, std::vector<float>> flines;


        void preprocessing(const cv::Mat &img, cv::Mat &dst);

        void histStretching(cv::Mat &img);

        void binarization(const cv::Mat &src, cv::Mat &dst);

        float hough(const cv::Mat &gray);

        void divide_lines(const std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &left_lines, std::vector<cv::Vec4i> &right_lines);

        void get_line_pos(std::vector<cv::Vec4i> &lines, bool left);

        void get_line_params(std::vector<cv::Vec4i> &lines, float &m, float &b);

        float run();

        bool check();
    };
    class MovingAverage
    {
    private:
        int samples;
        std::vector<float> data, weights;
        
    public:
        MovingAverage();
        MovingAverage(const int n);
        void add_sample(const float new_samples);
        float get_mm();
        float get_wmm();
    };
}

#endif