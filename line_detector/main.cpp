#include <iostream>
#include "opencv2/opencv.hpp"
#include <fstream>

#define maskminx 296
#define maskminy 18
#define maskmaxx 346
#define maskmaxy 20

void video(std::string filepath);
void preprocessing(const cv::Mat& frame, cv::Mat& dst);
void erase_lidar(cv::Mat& frame);
std::pair<int, int> connect_component(const cv::Mat& frame, std::vector<std::vector<int>>& ex_lines);

int main()
{
	std::string filepath = "video/Sub_project.avi";
	cv::Mat line_img = cv::imread("line.png", cv::IMREAD_GRAYSCALE);
	cv::Mat calibratied_frame;

	video(filepath);
}

void video(std::string filepath) {
	cv::Mat frame, roi_th_gray, roi;
	cv::VideoCapture cap(filepath);
	cv::TickMeter tm;
	int key = cv::waitKey(1);
	int f = 1;
	std::vector<std::vector<int>> line;
	std::pair<int, int> pos;

	std::ofstream csvfile;

	std::vector<std::vector<int>> ex_lines = { {0,0,0,0,0},{0,0,0,0,0} };//left,right

	tm.start();

	while (1) {
		if (!cap.isOpened()) {
			std::cerr << "can't open" << std::endl;
			return;
		}
		cap >> frame;
		if (frame.empty()) {
			std::cout << "Video end" << std::endl;
			break;
		}
		roi = frame(cv::Rect(cv::Point(0, 380), cv::Point(640, 400)));

		preprocessing(roi, roi_th_gray);
		pos = connect_component(roi_th_gray, ex_lines);

		cv::rectangle(frame, cv::Point(ex_lines[0][1], 380+ex_lines[0][2]), cv::Point(ex_lines[0][1] + ex_lines[0][3], 380+ex_lines[0][2] + ex_lines[0][4]), cv::Scalar(255, 0, 255), 2);
		cv::rectangle(frame, cv::Point(ex_lines[1][1], 380+ex_lines[1][2]), cv::Point(ex_lines[1][1] + ex_lines[1][3], 380+ex_lines[1][2] + ex_lines[1][4]), cv::Scalar(255, 255, 0), 2);
		
		cv::circle(frame, cv::Point(pos.first, 400), 1, cv::Scalar(255, 255, 255), 2);
		cv::circle(frame, cv::Point(pos.second, 400), 1, cv::Scalar(255, 255, 255), 2);
		cv::imshow("video", frame);
	
		if (cv::waitKey(25) == ' ') {
			break;
		}
		if (f % 30 == 0) {
			tm.stop();
			std::cout << f << " frame time: " << tm.getTimeMilli() << std::endl;
			tm.reset();
			tm.start();
			line.push_back({f, pos.first, pos.second});
		}
		f++;
		
	}
	cv::destroyAllWindows();
	if (f == 3287) {
		csvfile.open("result.csv", std::ios::out);
		for (int i = 0; i < line.size(); i++) {
			csvfile << line[i][0] << "," << line[i][1] << "," << line[i][2] << std::endl;
		}
		csvfile.close();
	}
}

void preprocessing(const cv::Mat& frame, cv::Mat &dst) {
	cv::GaussianBlur(frame, dst, cv::Size(3, 3), 0);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::Scalar mean = cv::mean(frame);
	double m = (mean[0] + mean[1] + mean[2]) / 3;
	dst = dst + (dst - m) * 1.0;
	cv::Mat dst_gray;
	std::vector<cv::Mat> sp;
	cv::cvtColor(dst, dst_gray, cv::COLOR_BGR2GRAY);
	cv::threshold(dst_gray, dst, 110, 255, cv::THRESH_BINARY_INV);
	erase_lidar(dst);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::threshold(dst, dst, 120, 255, cv::THRESH_BINARY);
}

std::pair<int, int> connect_component(const cv::Mat& frame, std::vector<std::vector<int>>& ex_lines) {
	cv::Mat dst;
	frame.copyTo(dst);
	cv::Mat compo, stats, centroids;
	std::vector<int> left_line = { 0,0,0,0,0 };
	std::vector<int> right_line = { 0,640,0,0,0 };
	int num = 0;
	num = cv::connectedComponentsWithStats(dst, compo, stats, centroids, 8, CV_32S);
	for (int i = 1; i < num; i++) {
		int area = stats.at<int>(i, cv::CC_STAT_AREA);
		int left = stats.at<int>(i, cv::CC_STAT_LEFT);
		int top = stats.at<int>(i, cv::CC_STAT_TOP);
		int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
		int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
		if (area < 150)
			continue;
		else if (area > 1200)
			continue;
		else if (left + width / 2 < 320 && area > 0) {
			if (left > left_line[1]) {
				left_line[0] = area;
				left_line[1] = left;
				left_line[2] = top;
				left_line[3] = width;
				left_line[4] = height;
			}
		}
		else if (left + width / 2 >= 320 && area > 0) {
			if (left < right_line[1]) {
				right_line[0] = area;
				right_line[1] = left;
				right_line[2] = top;
				right_line[3] = width;
				right_line[4] = height;
			}
		}
	}

	if (left_line[0] != 0 && right_line[0] != 0) {

	}
	else if (left_line[0] != 0) {
		right_line[0] = left_line[0];
		right_line[1] = left_line[1] + ex_lines[1][1] - ex_lines[0][1];
		right_line[2] = left_line[2];
		right_line[3] = (ex_lines[0][3] + left_line[3])/2;
		right_line[4] = left_line[4];
	}
	else if (right_line[0] != 0) {
		left_line[0] = right_line[0];
		left_line[1] = right_line[1] - (ex_lines[1][1] - ex_lines[0][1]);
		left_line[2] = right_line[2];
		left_line[3] = (ex_lines[1][3] + right_line[3])/2;
		left_line[4] = right_line[4];
	}
	else {
		left_line[0] = ex_lines[0][0];
		left_line[1] = ex_lines[0][1];
		left_line[2] = ex_lines[0][2];
		left_line[3] = ex_lines[0][3];
		left_line[4] = ex_lines[0][4];
		right_line[0] = ex_lines[1][0];
		right_line[1] = ex_lines[1][1];
		right_line[2] = ex_lines[1][2];
		right_line[3] = ex_lines[1][3];
		right_line[4] = ex_lines[1][4];
	}
	ex_lines[0] = left_line;
	ex_lines[1] = right_line;

	return { left_line[1] + left_line[3] / 4, right_line[1] + right_line[3] / 4 * 3 };
}

void erase_lidar(cv::Mat& frame) {
	cv::Mat lidar = (cv::Mat_<uchar>(maskmaxy-maskminy,maskmaxx-maskminx) << 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255) ;
	for (int y = maskminy; y < maskmaxy; y++)
		for (int x = maskminx; x < maskmaxx; x++)
			if (lidar.at<uchar>(y - maskminy, x - maskminx) == 255)
				frame.at<uchar>(y, x) = 0;
}
