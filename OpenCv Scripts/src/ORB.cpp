#include "ORB.h"
#include <iostream>
using namespace std;

Mat ORB_DETECTOR(Mat& img) {
    // Use modern OpenCV 4 ORB API
    cout << "ORB Feature Detector function called." << endl;
    Mat orb_features;

    int64 ORB_start = cv::getTickCount();
    // Create ORB detector (ORB is part of main modules in recent OpenCV)
    Ptr<ORB> detector = ORB::create();
    std::vector<KeyPoint> keypoints;
    detector->detect(img, keypoints);

    int64 ORB_end = cv::getTickCount();
    double ORB_time = (ORB_end - ORB_start) / cv::getTickFrequency();
    cout << "ORB Feature Detection Time: " << ORB_time << " seconds" << endl;

    // Draw keypoints into output image
    drawKeypoints(img, keypoints, orb_features, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imwrite("C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\ORB_Features_Output.png", orb_features); // Save the output image
    // while (1) {
    //     imshow("ORB Features", orb_features);
    //     char c = (char)waitKey(10);
    //     if (c != -1) break; //Press any key to stop program
    // }

    return orb_features;
}