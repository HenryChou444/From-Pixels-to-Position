#ifndef FFD_H
#define FFD_H

#include <opencv2/opencv.hpp>
using namespace cv;

std::vector<char> FAST_FEATURE_DETECTOR(Mat& img, const std::string& file_name = "");

#endif