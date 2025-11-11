#ifndef MS_Pyramid_H
#define MS_Pyramid_H

#include <opencv2/opencv.hpp>
using namespace cv;

std::vector<cv::Mat> MS_Pyramid(cv::Mat img, const int& levels);

#endif