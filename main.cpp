#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

using namespace cv;

int main()
{
	Mat img2;
	cv::Mat img = cv::imread("data/logo.png");
	cv::namedWindow("OpenCV");
	cv::imshow("OpenCV", img);
	cv::waitKey(0);
	waitKey(0);
	return 0;
}

