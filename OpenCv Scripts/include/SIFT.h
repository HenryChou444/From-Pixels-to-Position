#ifndef SIFT_H
#define SIFT_H

#include <opencv2/opencv.hpp>
using namespace cv;

Mat SIFT_DETECTOR(const Mat& img, const std::string& file_name = "");

#endif