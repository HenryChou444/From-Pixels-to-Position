#ifndef BRIEF_H
#define BRIEF_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "FFD.h"
using namespace cv;

std::vector<std::vector<int>> BRIEF_descriptors(cv::Mat img, const std::vector<Key_Point>& keypoints, const int& nbits, const int& s);

#endif