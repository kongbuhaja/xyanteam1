#define _USE_MATH_DEFINES

#include "opencv2/opencv.hpp"
#include "iostream"
#include "random"
#include <cmath>
#include <numeric>

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

const std::string toString(const std::vector<cv::Vec4i>& v);

const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 422.037858, 0.000000, 245.895397, 0.000000, 435.589734, 163.625535, 0.000000, 0.000000, 1.000000);
const cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.289296, 0.061035, 0.001786, 0.015238, 0.000000);

class MovingAverage
{
private:
    int samples;
    std::vector<float> data, weights;

public:
    MovingAverage()
    {
    }
    MovingAverage(const int n) : samples(n)
    {
        for (int i = 1; i < n + 1; i++)
        {
            weights.push_back(i);
        }
    }
    void add_sample(const float new_samples)
    {
        if (data.size() == samples)
            data.erase(data.begin(), data.begin() + 1);
        data.push_back(new_samples);
    }
    float get_mm()
    {
        return std::accumulate(data.begin(), data.end(), float(0));
    }

    float get_wmm()
    {
        float s = 0;
        for (int i = 0; i < data.size(); i++)
            s += data[i] * weights[i];
        return float(s) / std::accumulate(weights.begin(), weights.begin() + data.size(), float(0));
    }
};

class LineDetector
{
private:
    cv::Mat image;
    MovingAverage ma;

public:
    LineDetector()
    {
        ma = MovingAverage(MA_SIZE);
    }

    /*void cam_callback()
    {
       
        cv::Mat src = cv::Mat(480, 640, CV_8UC3, const_cast<uchar*>(&msg.data[0]), msg.step);
        
    }*/

    // void load(cv::Mat &img){
    //     img = images.back();
    //     images.pop_back();
    // }

    void preprocessing(const cv::Mat& img, cv::Mat& dst)
    {
        // load image
        cv::Mat undistorted_img, blur;
        // cv::cvtColor(img, undistorted_img, cv::COLOR_RGB2BGR);
        // undistorted_img=image;
        // image.copyTo(temp_image);
        //  undistort
        cv::undistort(img, dst, cameraMatrix, distCoeffs);
        // undistorted_img.copyTo(image);
        // image.copyTo(temp_image);
        // cv::Mat image2=image;
        //  stratching
        // cv::GaussianBlur(undistorted_img, blur, cv::Size(3,3), 0);
    }

    void binarization(const cv::Mat& src, cv::Mat& gray)
    {
        cv::Mat blur;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        cv::Scalar mean = cv::mean(gray);

        // gray = gray + (gray - mean[0]) * 6.0;
        // cv::GaussianBlur(gray,blur, cv::Size(3,3), 0);
        cv::threshold(gray, gray, 180, 255, cv::THRESH_BINARY_INV);
    }

    int hough(const cv::Mat& gray, cv::Mat& dst)
    {
        int lpos, rpos;
        cv::Mat edge_img, roi, roi_th, canny;
        std::vector<cv::Vec4i> lines, left_lines, right_lines;

        cv::Canny(gray, canny, LOW_THRESHOLD, HIGH_THRESHOLD);
        roi_th = canny(cv::Rect(0, OFFSET, WIDTH, GAP));
        cv::HoughLinesP(roi_th, lines, RHO, THETA, 10, 10, 3);
        dst = roi_th;
        if (lines.size() == 0)
        {
            return 320;
        }
        divide_lines(lines, left_lines, right_lines);
        std::cout << "LEFT LINES : " << toString(left_lines) << std::endl;
        std::cout << "RIGHT LINES : " << toString(right_lines) << std::endl;
        
        std::vector<float> left = get_line_pos(left_lines, true);
        std::vector<float> right = get_line_pos(right_lines, false);

        float mpos = (left[0] + right[0]) * 0.5;
        std::cout << left[0] << " " << right[0] << " ";
        return int(mpos);
        if (left[1] == right[1])
            mpos = int((left[0] + right[0]) / 2);
        else
        {
            mpos = (-(left[2] - right[2]) / (left[1] - right[1]));
            if (mpos > WIDTH)
                mpos = WIDTH - 1;
            else if (mpos < 0)
                mpos = 0;
        }

        ma.add_sample(mpos);

        float wmm = ma.get_wmm();
        return int(wmm - WIDTH * 0.5);
    }

    void divide_lines(const std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& left_lines, std::vector<cv::Vec4i>& right_lines)
    {
        std::vector<float> slopes;
        std::vector<cv::Vec4i> new_lines;
        float slope;

        for (int i = 0; i < lines.size(); i++)
        {
            if ((lines[i][2] - lines[i][0]) == 0)
            {
                slope = 0.0;
            }
            else
            {
                slope = float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);
            }

            if ((slope > -SLOPE_THRESHOLD) && (slope < SLOPE_THRESHOLD))
            {
                slopes.push_back(slope);
                new_lines.push_back(lines[i]);
                std::cout << "slope :" << slope << std::endl;
            }
        }

        for (int i = 0; i < slopes.size(); i++)
        {
            std::cout << "slopes[i] : " << slopes[i] << std::endl;
            std::cout << "new_lines[i][2] : " << new_lines[i][2] << std::endl;
            std::cout << "new_lines[i][0] : " << new_lines[i][0] << std::endl;
            if ((slopes[i] < 0) && (new_lines[i][2] < WIDTH / 2))
            {
                left_lines.push_back(new_lines[i]);
            }
            else if ((slopes[i] > 0) && (new_lines[i][0] > WIDTH / 2))
            {
                right_lines.push_back(new_lines[i]);
            }
        }
    }

    std::vector<float> get_line_pos(std::vector<cv::Vec4i>& lines, bool left)
    {
        float& m = *new float();
        float& b = *new float();
        get_line_params(lines, m, b);
        int pos;
        if (m == 0 && b == 0)
        {
            std::cout << "OOPS! " << (left ? "LEFT" : "RIGHT") << "! m and b both are zero." << std::endl;
            if (left)
                pos = 0;
            if (!left)
                pos = WIDTH;
        }
        else
        {
            int y = GAP / 2;
            pos = (y - b) / m;
            // float x1 = (height -b) / float(m);
            // float x2 = (height/2 -b) / float(m);
        }
        std::vector<float> result = { float(pos), m, b };
        std::cout << "m:" << m << " pos: " << pos << std::endl;
        return result;
    }

    void get_line_params(std::vector<cv::Vec4i>& lines, float& m, float& b)
    {
        float x_sum = 0.0;
        float y_sum = 0.0;
        float m_sum = 0.0;

        if (lines.size() == 0)
        {
            std::cout << "OOPS! Lines are empty." << std::endl;

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

    int run(cv::Mat& src, cv::Mat& output_show)
    {
        cv::Mat dst, th_img, result;
        int cte = 90;

        cv::cvtColor(src, image, cv::COLOR_RGB2BGR);

        preprocessing(image, dst);
        binarization(dst, th_img);
        cte = hough(th_img, result);
        output_show = result;
        return cte;
    }

    bool check()
    {
        return image.cols != 640;
    }
};

int main(int argc, char** argv)
{
    int error;
    LineDetector* line_detector = new LineDetector();
    cv::Mat output_show;
    cv::Mat frame;
    cv::Mat src;
    cv::VideoCapture cap("Sub_project.avi");

    if (!cap.isOpened()) {
        std::cerr << "Video open failed!" << std::endl;
        return -1;
    }
    while (1) {
        cap >> src;
        error = line_detector->run(src, output_show);
        
        cv::cvtColor(output_show, output_show, cv::COLOR_GRAY2BGR);

        cv::circle(output_show, cv::Point(error, 10), 10, cv::Scalar(255, 0, 255));
        cv::circle(output_show, cv::Point(WIDTH / 2, 10), 10, cv::Scalar(0, 255, 255));

        cv::imshow("output_show", output_show);
        std::cout << "error : " << error << std::endl;


        if (cv::waitKey(30) == 27)
            break;
    }
    cap.release();
    cv::destroyAllWindows();

    delete line_detector;
    return 0;
}


const std::string toString(const std::vector<cv::Vec4i>& v) {
    std::string result = "";
    for (int i = 0; i < v.size(); i++) {
        result += "\n[" + std::to_string(i) + "]\n" +
            "[0]:" + std::to_string(v[i][0]) +
            "[1]:" + std::to_string(v[i][1]) +
            "[2]:" + std::to_string(v[i][2]) +
            "[3]:" + std::to_string(v[i][3]) + "\n";
    }
    return result;
}