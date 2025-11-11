#include "SIFT.h"
#include <opencv2/features2d.hpp>
#include <iostream>
using namespace std;

Mat SIFT_DETECTOR(const Mat& img, const std::string& file_name) {
    // Use modern OpenCV 4 SIFT API
    cout << "SIFT Feature Detector function called." << endl;
    Mat sift_features;

    int64 SIFT_start = cv::getTickCount();
    // Create SIFT detector (SIFT is part of main modules in recent OpenCV)
    Ptr<SIFT> detector = SIFT::create();
    std::vector<KeyPoint> keypoints;
    detector->detect(img, keypoints);
    
    int64 SIFT_end = cv::getTickCount();
    double SIFT_time = (SIFT_end - SIFT_start) / cv::getTickFrequency();
    cout << "SIFT Feature Detection Time: " << SIFT_time << " seconds" << endl;


    // Draw keypoints into output image
    drawKeypoints(img, keypoints, sift_features, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string full_path = "C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\SIFT Results\\" + file_name;
    cout << "Saving SIFT features to: " << full_path << endl;
    imwrite(full_path, sift_features); // Save the output image
    // while (1) {
    //     imshow("SIFT Features", sift_features);
    //     char c = (char)waitKey(10);
    //     if (c != -1) break; //Press any key to stop program
    // }

    return sift_features;
}