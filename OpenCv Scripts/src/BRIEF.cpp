#include "BRIEF.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <random>
using namespace std;

std::vector<std::vector<int>> BRIEF_descriptors(cv::Mat img, const std::vector<Key_Point>& keypoints, const int& nbits, const int& s) { // nbits: number of bits in descriptor, s: patch size
    cout << "Creating descriptors for keypoints." << endl;
    std::vector<std::vector<int>> descriptors; // Vector to hold descriptors
    std::mt19937 rng(12345); // define a seed for the random number generator
    std::uniform_int_distribution<int> dist(-s/2, s/2);
    std::vector<std::pair<cv::Point, cv::Point>> description_pattern;


    for (int i = 0; i < nbits; ++i) {   // Generate the random points for descriptor
        cv::Point p1(dist(rng), dist(rng));
        cv::Point p2(dist(rng), dist(rng));
        description_pattern.push_back({p1, p2});
    }

    // Create an image of the description pattern for visualization
    
    // cv::Mat pattern_img = cv::Mat::zeros(s, s, CV_8UC1);
    // for (int i = 0; i < nbits; ++i) {
    //     //imshow the pattern points on a blank image for visualization
    //     //draw lines between the points with random colours
    //     cv::line(pattern_img, description_pattern[i].first + cv::Point(s/2, s/2), description_pattern[i].second + cv::Point(s/2, s/2), cv::Scalar(rand() % 256, rand() % 256, rand() % 256), 1);
    // }

    // while (1) {
    //     cv::imshow("Pattern Points", pattern_img);
    //     char c = (char)cv::waitKey(10);
    //     if (c != -1) break; //Press escape to stop program
    //     // Save image
    //     imwrite("C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\BRIEF_Description_Pattern.png", pattern_img);
    // }

    for (int i = 0; i < (int)keypoints.size(); ++i) {
        std::vector<int> descriptor; // Descriptor for current keypoint
        cv::Point kp = keypoints[i].pt;

        for (int j = 0; j < nbits; j++){
            int x1 = description_pattern[j].first.x;
            int y1 = description_pattern[j].first.y;
            int x2 = description_pattern[j].second.x;
            int y2 = description_pattern[j].second.y;
            // Check if random points are inside of the boundaries 
            if (kp.x + x1 < 0 || kp.x + x1 >= img.cols || kp.y + y1 < 0 || kp.y + y1 >= img.rows ||
                kp.x + x2 < 0 || kp.x + x2 >= img.cols || kp.y + y2 < 0 || kp.y + y2 >= img.rows) {
                descriptor.push_back(0); 
                continue;
            }

            int p1 = img.at<uchar>(kp.y + y1, kp.x + x1);
            int p2 = img.at<uchar>(kp.y + y2, kp.x + x2);
            if (p1 < p2){
                descriptor.push_back(1);
            }
            else {
                descriptor.push_back(0);
            }
        }
        
        descriptors.push_back(descriptor);
    }
    
    return descriptors;
}