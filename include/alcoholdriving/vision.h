#ifndef MOTOR_H_
#define MOTOR_H_

#include "ros/ros.h"
#include "iostream"
#include "random"
#include "opencv2/opencv.hpp"

#include <cmath>
#include <numeric>
namespace alcoholdriving
{
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