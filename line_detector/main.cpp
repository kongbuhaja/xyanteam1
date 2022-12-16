#include <iostream>
#include "opencv2/opencv.hpp"
#include <fstream>

#define maskminx 296
#define maskminy 18
#define maskmaxx 346
#define maskmaxy 20

void video(std::string filepath);
void preprocessing(const cv::Mat& frame, cv::Mat& dst);
void houghtransform(const cv::Mat& frame);
void erase_lidar(cv::Mat& frame);
void contour(const cv::Mat& frame);
std::pair<int, int> connect_component(const cv::Mat& frame, std::vector<std::vector<int>>& ex_lines);

int main()
{
	std::string filepath = "video/Sub_project.avi";
	cv::Mat line_img = cv::imread("line.png", cv::IMREAD_GRAYSCALE);
	std::vector<cv::Point> origin_point{ cv::Point(77, 0), cv::Point(542, 0), cv::Point(48, 19), cv::Point(566, 19) };
	std::vector<cv::Point> trans_point{ cv::Point(48, 0), cv::Point(566, 0), cv::Point(48, 19), cv::Point(540, 19) };
	

	//erase_lidar(line_img);
	//houghtransform(line_img);
	//cv::waitKey();
	//cv::destroyAllWindows();

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
		}
		cap >> frame;
		if (frame.empty()) {
			std::cout << "Video end" << std::endl;
			break;
		}
		roi = frame(cv::Rect(cv::Point(0, 380), cv::Point(640, 400)));

		preprocessing(roi, roi_th_gray);
		//houghtransform(dst);
		pos = connect_component(roi_th_gray, ex_lines);
		contour(roi_th_gray);


		cv::rectangle(frame, cv::Point(ex_lines[0][1], 380+ex_lines[0][2]), cv::Point(ex_lines[0][1] + ex_lines[0][3], 380+ex_lines[0][2] + ex_lines[0][4]), cv::Scalar(255, 0, 255), 2);
		cv::rectangle(frame, cv::Point(ex_lines[1][1], 380+ex_lines[1][2]), cv::Point(ex_lines[1][1] + ex_lines[1][3], 380+ex_lines[1][2] + ex_lines[1][4]), cv::Scalar(255, 255, 0), 2);
		
		cv::imshow("video", frame);
		cv::imshow("roi_th_gray", roi_th_gray);
	
		if (cv::waitKey(1) == ' ') {
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
	cv::Mat th, YCrCb, Y, Cr, Cb, gray, Yth, dst_gray;
	std::vector<cv::Mat> sp;
	cv::cvtColor(dst, dst_gray, cv::COLOR_BGR2GRAY);
	//cv::cvtColor(dst, YCrCb, cv::COLOR_BGR2YCrCb);
	cv::threshold(dst_gray, dst, 110, 255, cv::THRESH_BINARY_INV);
	erase_lidar(dst);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::threshold(dst, dst, 120, 255, cv::THRESH_BINARY);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::threshold(dst, dst, 120, 255, cv::THRESH_BINARY);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
	cv::threshold(dst, dst, 120, 255, cv::THRESH_BINARY);
	//cv::split(YCrCb, sp);
	//Y = sp[0];
	//Cr = sp[1];
	//Cb = sp[2];
	//cv::threshold(Y, Yth, 120, 255, cv::THRESH_BINARY_INV);

	//cv::imshow("Y", Y);
	//cv::imshow("Yth", Yth);
	cv::imshow("roi", frame);
	cv::imshow("gray_th", dst);
}

std::pair<int, int> connect_component(const cv::Mat& frame, std::vector<std::vector<int>>& ex_lines) {
	cv::Mat dst;
	frame.copyTo(dst);
	cv::Mat compo, stats, centroids;
	std::vector<int> left_line = { 0,0,0,0,0 };
	std::vector<int> right_line = { 0,0,0,0,0 };
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
		else if (left + width / 2 < 320 && area > 0 && left_line[1]>left) {
			left_line[0] = area;
			left_line[1] = left;
			left_line[2] = top;
			left_line[3] = width;
			left_line[4] = height;
		}
		else if (left + width / 2 >= 320 && area > 0 && right_line[1] < left) {
			right_line[0] = area;
			right_line[1] = left;
			right_line[2] = top;
			right_line[3] = width;
			right_line[4] = height;
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

	cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	cv::rectangle(dst, cv::Point(left_line[1], left_line[2]), cv::Point(left_line[1] + left_line[3], left_line[2] + left_line[4]), cv::Scalar(255, 255, 0), 1);
	cv::rectangle(dst, cv::Point(right_line[1], right_line[2]), cv::Point(right_line[1] + right_line[3], right_line[2] + right_line[4]), cv::Scalar(255, 255, 0), 1);
	cv::circle(dst, cv::Point(left_line[1] + left_line[3] / 4, 20), 1, cv::Scalar(0, 0, 0),2);
	cv::circle(dst, cv::Point(right_line[1] + right_line[3] / 4*3, 20), 1, cv::Scalar(0, 0, 0),2);

	cv::imshow("component", dst);

	return { left_line[1] + left_line[3] / 4, right_line[1] + right_line[3] / 4 * 3 };
}
void houghtransform(const cv::Mat& frame) {
	cv::Mat edge;
	cv::Mat frame2 = frame;
	cv::Mat dst = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC3);

	cv::Canny(frame, edge, 100, 150);
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(edge, lines, 1, CV_PI / 180, 0, 0, 1);

	for (size_t i = 0; i < lines.size(); i++) {
		cv::Vec4i line = lines[i];
		//cv::line(dst, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		//std::cout << line[0] << " " << line[1] << " " << line[2] << " " << line[3] << std::endl;
	}


	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(frame2, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	for (int idx = 0; idx < contours.size(); idx++) {
		cv::Scalar color(255, 0, 0);
		cv::drawContours(dst, contours, idx, color, 2, cv::LINE_8, hierarchy);
	}


	cv::imshow("hough", dst);
	cv::imshow("canny", edge);
}

void contour(const cv::Mat& frame) {
	cv::Mat img = frame;
	cv::Mat dst = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC3);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	for (int idx = 0; idx < contours.size(); idx++) {
		cv::Scalar color(255, 0, 0);
		cv::drawContours(dst, contours, idx, color, 2, cv::LINE_8, hierarchy);
	}

	cv::Point lposl, lposr, rposl, rposr;
	
	for (int i = 0; i < contours.size(); i++) {
		for (int j = 0; j < contours[i].size(); j++) {
			cv::circle(dst, cv::Point(contours[i][j].x, contours[i][j].y), 1, cv::Scalar(0, 255, 0), 1);
		}
	}

	cv::imshow("contour", dst);
}

void erase_lidar(cv::Mat& frame) {
	cv::Mat lidar = (cv::Mat_<uchar>(maskmaxy-maskminy,maskmaxx-maskminx) << 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255) ;
	for (int y = maskminy; y < maskmaxy; y++)
		for (int x = maskminx; x < maskmaxx; x++)
			if (lidar.at<uchar>(y - maskminy, x - maskminx) == 255)
				frame.at<uchar>(y, x) = 0;
}
