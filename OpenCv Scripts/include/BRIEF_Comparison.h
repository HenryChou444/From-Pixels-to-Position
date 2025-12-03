#ifndef BRIEF_COMPARISON_H
#define BRIEF_COMPARISON_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "FFD.h"
using namespace cv;

std::vector<std::vector<int>> BRIEF_Comparison(const std::vector<Key_Point>& keypoints1, const std::vector<Key_Point>& keypoints2, const std::vector<std::vector<int>>& descriptors1, const std::vector<std::vector<int>>& descriptors2);

#endif