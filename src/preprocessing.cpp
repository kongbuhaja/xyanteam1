#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include <cmath>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "alcoholdriving/vision.h"

const std::string toString(const std::vector<cv::Vec4i> &v);

const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 422.037858, 0.000000, 245.895397, 0.000000, 435.589734, 163.625535, 0.000000, 0.000000, 1.000000);
const cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.289296, 0.061035, 0.001786, 0.015238, 0.000000);

alcoholdriving::LineDetector::LineDetector(ros::NodeHandle nh) : nh_(nh)
{
    ma = MovingAverage(MA_SIZE);
}

alcoholdriving::LineDetector::~LineDetector()
{
    /* TODO Destroy Objects in LineDetector*/
}

void alcoholdriving::LineDetector::preprocessing(const cv::Mat &img, cv::Mat &dst)
{
    cv::Mat undistorted_img; 
    cv::undistort(img, undistorted_img, cameraMatrix, distCoeffs);
    cv::cvtColor(undistorted_img, undistorted_img, cv::COLOR_BGR2GRAY);
    dst = undistorted_img(cv::Rect(0, OFFSET, WIDTH, GAP));
}

void alcoholdriving::LineDetector::histStretching(cv::Mat &img){
    int gmin, gmax;
        gmin = img.at<uchar>(0,0);
        gmax = gmin;
        for(int y=0; y<img.rows; y++){
            for(int x=0; x<img.cols; x++){
                int value = img.at<uchar>(y,x);
                if(value<gmin) gmin=value;
                else if(value>gmax) gmax=value;
            }
        }
        for(int y=0; y<img.rows; y++){
            for(int x=0; x<img.cols; x++){
                img.at<uchar>(y,x) = (img.at<uchar>(y,x) - gmin)*255 / (gmax - gmin);
            }
        }
}

void alcoholdriving::LineDetector::binarization(const cv::Mat &src, cv::Mat &dst)
{
    cv::Mat th_img = src;
    histStretching(th_img);
    cv::threshold(th_img, th_img, 200, 255, cv::THRESH_BINARY);
    dst = th_img;
    // gray = gray + (gray - mean[0]) * 6.0;
    // cv::GaussianBlur(gray,blur, cv::Size(3,3), 0);
}

float alcoholdriving::LineDetector::hough(const cv::Mat &th_img)
{
    int lpos, rpos, mpos;
    cv::Mat edge_img, roi, roi_th, canny;
    std::vector<cv::Vec4i> lines, left_lines, right_lines;
    
    cv::Canny(th_img, canny, LOW_THRESHOLD, HIGH_THRESHOLD);
    roi_th = canny(cv::Rect(0, OFFSET, WIDTH, GAP));
    cv::HoughLinesP(roi_th, lines,RHO, THETA, 10, 10, 3);

    divide_lines(lines, left_lines, right_lines);
    
    get_line_pos(left_lines, true);
    get_line_pos(right_lines, false);
       
    if(flines.first[1]==flines.second[1])
        mpos=int((flines.first[0]+flines.second[0])/2);
    else{
        mpos = (-(flines.first[2]-flines.second[2])/(flines.first[1]-flines.second[1]));
        if(mpos>=WIDTH)
            mpos=WIDTH-1;
        else if(mpos<0)
            mpos=0;
    }

    ma.add_sample(mpos);
        
    return (ma.get_wmm() - WIDTH * 0.5)/(WIDTH*2);
}

void alcoholdriving::LineDetector::divide_lines(const std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &left_lines, std::vector<cv::Vec4i> &right_lines)
{
    std::vector<float> slopes;
    std::vector<cv::Vec4i> new_lines;
    float slope;
        
    if(lines.size()==0){
        left_lines = {};
        right_lines = {};
        return;
    }

    for(int i=0; i<lines.size(); i++){
        if((lines[i][2] - lines[i][0])==0){slope = 0.0;}
        else{slope = float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);}

        if((slope > -SLOPE_THRESHOLD) && (slope < SLOPE_THRESHOLD)){
            slopes.push_back(slope);
            new_lines.push_back(lines[i]);
        }
    }

    for(int i=0; i<slopes.size(); i++){
        if((slopes[i] < 0) && (new_lines[i][2] < WIDTH/2)) {left_lines.push_back(new_lines[i]);}
        else if((slopes[i]) > 0 && (new_lines[i][0] > WIDTH/2)) {right_lines.push_back(new_lines[i]);}
    }
}

void alcoholdriving::LineDetector::get_line_pos(std::vector<cv::Vec4i> &lines, bool left)
{
    float m, b;
    get_line_params(lines, m, b);
    int pos;
    if(m==0 && b==0){
        if(left) pos=0;
        if(!left) pos=WIDTH;
    }
    else{
        int y = GAP/2;
        pos = (y-b) / m;
        if(left) flines.first = {float(pos), m, b};
        else flines.second = {float(pos), m, b};
    }
}

void alcoholdriving::LineDetector::get_line_params(std::vector<cv::Vec4i> &lines, float &m, float &b)
{
    float x_sum = 0.0;
    float y_sum = 0.0;
    float m_sum = 0.0;

    if (lines.size() == 0)
    {
        m = 0;
        b = 0;
        return;
    }

    for (int i = 0; i < lines.size(); i++)
    {
        x_sum += lines[i][0] + lines[i][2];
        y_sum += lines[i][1] + lines[i][3];
        m_sum += float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);
    }
    float x_avg = x_sum / (lines.size() * 2);
    float y_avg = y_sum / (lines.size() * 2);
    m = m_sum / lines.size();
    b = y_avg - m * x_avg;
}

float alcoholdriving::LineDetector::run()
{
    cv::Mat dst, th_img;
    float error=0;
       
    preprocessing(image, dst);
    binarization(dst, th_img);
    error = hough(th_img);

    return error;
}

bool alcoholdriving::LineDetector::check()
{
    return image.cols != 640;
}