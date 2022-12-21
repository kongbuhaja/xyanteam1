#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include <cmath>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#define PI M_PI
#define low_threshold 60
#define high_threshold 70
#define slope_threshold 10
#define offset 380
#define roi_height 20
#define width 640
#define height 480
#define rho 1
#define theta PI/180
#define limit_slope 10000
#define images_size 30
#define ma_size 50


const cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 422.037858, 0.000000, 245.895397, 0.000000, 435.589734, 163.625535, 0.000000, 0.000000, 1.000000);
const cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.289296, 0.061035, 0.001786, 0.015238, 0.000000);

class MovingAverage{
private:
    int samples;
    std::vector<float> data, weights;
public:
    MovingAverage(){
    }
    MovingAverage(const int n): samples(n){
        for(int i=1; i<n+1; i++){
            weights.push_back(i);
        }
    }
    void add_sample(const float new_samples){
        if(data.size() ==samples)
            data.erase(data.begin(),data.begin()+1);
        data.push_back(new_samples);
    }
    float get_mm(){
        return std::accumulate(data.begin(), data.end(), float(0));
    }

    float get_wmm(){
        float s=0;
        for(int i=0; i<data.size(); i++)
            s += data[i] * weights[i];
        return float(s) / std::accumulate(weights.begin(), weights.begin() + data.size(), float(0));
    }
};

class Line_Detector{
private:
    int gap;
    ros::NodeHandle nh;
    ros::Subscriber cam_sub;
    cv::Mat image;
    MovingAverage ma;
public:
    Line_Detector(const ros::NodeHandle &nh_): nh(nh_){
        cam_sub = nh.subscribe("/usb_cam/image_raw", 1, &Line_Detector::cam_callback, this);
        ma = MovingAverage(ma_size);
    }

    void cam_callback(const sensor_msgs::Image &msg){
        cv::Mat src = cv::Mat(480, 640, CV_8UC3, const_cast<uchar *>(&msg.data[0]), msg.step);
        cv::cvtColor(src, image, cv::COLOR_RGB2BGR);
    }

    // void load(cv::Mat &img){
    //     img = images.back();
    //     images.pop_back();
    // }

    void preprocessing(const cv::Mat &img, cv::Mat &dst){
        // load image
        cv::Mat undistorted_img, blur;
        //cv::cvtColor(img, undistorted_img, cv::COLOR_RGB2BGR);
        //undistorted_img=image;
        //image.copyTo(temp_image);
        // undistort
        cv::undistort(img, dst, cameraMatrix, distCoeffs);
        //undistorted_img.copyTo(image);
        //image.copyTo(temp_image);
        //cv::Mat image2=image;
        // stratching
        //cv::GaussianBlur(undistorted_img, blur, cv::Size(3,3), 0);
    }

    void binarization(const cv::Mat &src, cv::Mat &gray){
        cv::Mat blur;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        gray = gray + (gray - 128) * 1.0;
        //cv::GaussianBlur(gray,blur, cv::Size(3,3), 0);
        //cv::threshold(gray, gray, 110, 255, cv::THRESH_BINARY_INV);
    }

    int hough(const cv::Mat &gray, cv::Mat &dst){
        int lpos, rpos;
        cv::Mat edge_img, roi, roi_th;
        std::vector<cv::Vec4i> lines, left_lines, right_lines;
        
        roi = gray(cv::Rect(0, offset, width, roi_height));
        cv::Canny(roi, roi_th, low_threshold, high_threshold);
        
        cv::HoughLinesP(roi_th, lines, rho, theta, 30, 30, 10);
        
        if(lines.size()==0){
            return 320;
        }
        divide_lines(lines, left_lines, right_lines);
        // ROS_ERROR("3");
        std::vector<float> left = get_line_pos(left_lines, true);
        std::vector<float> right = get_line_pos(right_lines, false);
        float mpos = (left[0] + right[0]) * 0.5;

        // if(left[1]==right[1])
        //     mpos=int((left[0]+right[0])/2);
        // else{
        //     mpos = (-(left[2]-right[2])/(left[1]-right[1]));
        //     if(mpos>width)
        //         mpos=width-1;
        //     else if(mpos<0)
        //         mpos=0;
        // }

        ma.add_sample(mpos);
        dst=gray;
        float wmm=ma.get_wmm();
        return (wmm - width * 0.5);
    }

    void divide_lines(const std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &left_lines, std::vector<cv::Vec4i> &right_lines){
        std::vector<float> slopes;
        std::vector<cv::Vec4i> new_lines;
        float slope;

        for(int i=0; i<lines.size(); i++){
            if((lines[i][2] - lines[i][0])==0){slope = limit_slope;}
            else{slope = float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);}

            if((slope > -slope_threshold) && (slope < slope_threshold)){
                slopes.push_back(slope);
                new_lines.push_back(lines[i]);
            }
        }

        for(int i=0; i<slopes.size(); i++){
            if((slopes[i] < 0) && (new_lines[i][2] < width/2 - 90)) {left_lines.push_back(new_lines[i]);}
            else if((slopes[i]) > 0 && (new_lines[i][0] < width/2 + 90)) {right_lines.push_back(new_lines[i]);}
        }
    }

    std::vector<float> get_line_pos(std::vector<cv::Vec4i> &lines, bool left){
        float m, b;
        get_line_params(lines, m, b);
        int pos;
        if(m==0 && b==0){
            if(left) pos=0;
            if(!left) pos=width;
        }
        else{
            int y = gap/2;
            pos = (y-b) / m;
            // float x1 = (height -b) / float(m);
            // float x2 = (height/2 -b) / float(m);
        }
        std::vector<float> result={float(pos),m,b};
        return result;
    }

    void get_line_params(std::vector<cv::Vec4i> &lines, float &m, float &b){
        float x_sum = 0.0;
        float y_sum = 0.0;
        float m_sum = 0.0;

        if(lines.size()==0){
            m=0; b=0;
            return;
        }

        for(int i=0; i<lines.size(); i++){
            x_sum += lines[i][0] + lines[i][2];
            y_sum += lines[i][1] + lines[i][3];
            m_sum += float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);
        }
        float x_avg = x_sum / (lines.size() * 2);
        float y_avg = y_sum / (lines.size() * 2);
        m = m_sum / lines.size();
        b = y_avg - m * x_avg;
    }

    float run(){
        cv::Mat dst, th_img;
        int cte=90;
        
        preprocessing(image, dst);
        binarization(dst, th_img);
        cte=hough(th_img, dst);
        return cte;
    }

    int check(){
        if(image.cols!=640)
            return 1;
        return 0;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle nh_;

    ros::Rate rate(30);
    Line_Detector line_detector(nh_);

    while(ros::ok()){
        ros::spinOnce();
        if(line_detector.check()==1)
            continue;
        line_detector.run();
    }
    return 0;
}
