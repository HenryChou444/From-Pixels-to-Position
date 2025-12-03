#ifndef FFD_H
#define FFD_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace cv;

struct Key_Point{ 
    Point pt;
    int score;
};

// Declare FAST_FEATURE_DETECTOR after Key_Point is known
std::vector<Key_Point> FAST_FEATURE_DETECTOR(Mat& img, const std::string& file_name = "");

#endif