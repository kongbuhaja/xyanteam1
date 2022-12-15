#include <iostream>
#include "opencv2/opencv.hpp"

void video(std::string filepath);
void preprocessing(const cv::Mat& frame, cv::Mat& dst);

int main()
{
	std::string filepath = "video/Sub_project.avi";
	cv::Mat frame;
	cv::VideoCapture cap(filepath);
	
	video(filepath);
}

void video(std::string filepath) {
	cv::Mat frame, dst, roi;
	cv::VideoCapture cap(filepath);
	cv::TickMeter tm;
	int key = cv::waitKey(1);
	int f = 1;
	tm.start();

	while (1) {
		if (!cap.isOpened()) {
			std::cerr << "can't open" << std::endl;
		}
		cap >> frame;
		if (frame.empty()) {
			std::cout << "Video end" << std::endl;
			return;
		}
		roi = frame(cv::Rect(cv::Point(0, 380), cv::Point(640, 400)));
		preprocessing(roi, dst);

		cv::imshow("video", frame);
		cv::imshow("dst", dst);
	
		if (cv::waitKey(1) == ' ') {
			break;
		}
		if (f == 30) {
			f = 0;
			tm.stop();
			std::cout << "time: " << tm.getTimeMilli() << std::endl;
			tm.reset();
			tm.start();
		}
		else {
			f += 1;
		}
	}
	cv::destroyAllWindows();
}

void preprocessing(const cv::Mat& frame, cv::Mat &dst) {
	cv::Mat img = frame;
	cv::GaussianBlur(img, dst, cv::Size(3, 3), 0);
	int m = ((int)cv::mean(frame)[0] + (int)cv::mean(frame)[1] + (int)cv::mean(frame)[2]) / 3;
	dst = dst + (dst - m) * 1.0;
	cv::Mat th, YCrCb, Y, Cr, Cb, gray, Yth, dst_gray;
	std::vector<cv::Mat> sp;
	cv::cvtColor(dst, dst_gray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(dst, YCrCb, cv::COLOR_BGR2YCrCb);
	cv::threshold(dst_gray, th, 120, 255, cv::THRESH_BINARY_INV);
	cv::split(YCrCb, sp);
	Y = sp[0];
	//Cr = sp[1];
	//Cb = sp[2];
	cv::threshold(Y, Yth, 120, 255, cv::THRESH_BINARY_INV);

	cv::imshow("Y", Y);
	cv::imshow("Yth", Yth);
	cv::imshow("gray_th", th);
}