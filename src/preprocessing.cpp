#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include <cmath>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "alcoholdriving/vision.h"

#define PI M_PI
#define low_threshold 60
#define high_threshold 70
#define slope_threshold 10
#define offset 380
#define gap 20
#define width 640
#define height 480
#define rho 1
#define theta PI/180
#define limit_slope 10
#define images_size 30
#define ma_size 50


const cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 422.037858, 0.000000, 245.895397, 0.000000, 435.589734, 163.625535, 0.000000, 0.000000, 1.000000);
const cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.289296, 0.061035, 0.001786, 0.015238, 0.000000);


class Line_Detector{
private:
    ros::NodeHandle nh;
    ros::Subscriber cam_sub;
    cv::Mat image;
    alcoholdriving::MovingAverage ma;
public:
    Line_Detector(const ros::NodeHandle &nh_): nh(nh_){
        cam_sub = nh.subscribe("/usb_cam/image_raw", 1, &Line_Detector::cam_callback, this);
        ma = alcoholdriving::MovingAverage(ma_size);
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
        cv::Scalar mean = cv::mean(gray);
        
        //gray = gray + (gray - mean[0]) * 6.0;
        //cv::GaussianBlur(gray,blur, cv::Size(3,3), 0);
        cv::threshold(gray, gray, 180, 255, cv::THRESH_BINARY_INV);
    }

    int hough(const cv::Mat &gray, cv::Mat &dst){
        int lpos, rpos;
        cv::Mat edge_img, roi, roi_th, canny;
        std::vector<cv::Vec4i> lines, left_lines, right_lines;
        
        
        cv::Canny(gray, canny, low_threshold, high_threshold);
        roi_th = canny(cv::Rect(0, offset, width, gap));
        cv::HoughLinesP(roi_th, lines, rho, theta, 10, 10, 3);
        dst=roi_th;
        if(lines.size()==0){
            return 320;
        }
        divide_lines(lines, left_lines, right_lines);
    
        std::vector<float> left = get_line_pos(left_lines, true);
        std::vector<float> right = get_line_pos(right_lines, false);
        float mpos = (left[0] + right[0]) * 0.5;
        std::cout<<left[0]<<" "<<right[0]<<" ";
        return int(mpos);
        if(left[1]==right[1])
            mpos=int((left[0]+right[0])/2);
        else{
            mpos = (-(left[2]-right[2])/(left[1]-right[1]));
            if(mpos>width)
                mpos=width-1;
            else if(mpos<0)
                mpos=0;
        }

        ma.add_sample(mpos);
        
        float wmm=ma.get_wmm();
        return int(wmm - width * 0.5);
    }

    void divide_lines(const std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &left_lines, std::vector<cv::Vec4i> &right_lines){
        std::vector<float> slopes;
        std::vector<cv::Vec4i> new_lines;
        float slope;

        for(int i=0; i<lines.size(); i++){
            if((lines[i][2] - lines[i][0])==0){slope = 0.0;}
            else{slope = float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);}

            if((slope > -slope_threshold) && (slope < slope_threshold)){
                slopes.push_back(slope);
                new_lines.push_back(lines[i]);
                std::cout << "slope :" <<slope <<std::endl;
            }
        }

        for(int i=0; i<slopes.size(); i++){
            if((slopes[i] < 0) && (new_lines[i][2] < width/2)) {left_lines.push_back(new_lines[i]);}
            else if((slopes[i]) > 0 && (new_lines[i][0] < width/2)) {right_lines.push_back(new_lines[i]);}
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
        std::cout<<"m:"<<m<<" pos: "<<pos<<std::endl;
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

    int run(cv::Mat &output_show){
        cv::Mat dst, th_img, result;
        int cte=90;
        
        preprocessing(image, dst);
        binarization(dst, th_img);
        cte=hough(th_img, result);
        output_show=result;
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
    int error;

    ros::Rate rate(30);
    Line_Detector line_detector(nh_);
    cv::Mat output_show;
    while(ros::ok()){
        ros::spinOnce();
        if(line_detector.check()==1)
            continue;
        error=line_detector.run(output_show);
        cv::cvtColor(output_show, output_show, cv::COLOR_GRAY2BGR);
        //cv::circle(output_show, cv::Point(error+320, 0), 2, cv::Scalar(255,255,255), 3);
        cv::imshow("output_show", output_show);
        std::cout<<error<<std::endl;
        if(cv::waitKey(10) == 27){
            break;
        }        
    }
    cv::destroyAllWindows();
    return 0;
}
