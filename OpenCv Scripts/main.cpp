#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp> 
#include "FFD.h"
#include "MS_Pyramid.h"
#include "Gaussian_Kernel_Gen.h"
#include "SIFT.h"
#include "ORB.h"
using namespace cv;
using namespace std;



int main()
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR); // Suppress info/warning logs

    namespace fs = std::filesystem;
    const fs::path data_dir = R"(C:\Users\Alec\Documents\DTU\E25\Image Analysis with Microcomputer\Final Project\From-Pixels-to-Position\Data\0_to_14_cm_v50mms\Image_data)";

    Mat im_pyramid_test = imread(R"(C:\Users\Alec\Documents\DTU\E25\Image Analysis with Microcomputer\Exercises\Image-Analysis-with-Microcomputer-Exercises\PIC1_L.png)", IMREAD_GRAYSCALE);
    if (im_pyramid_test.empty()) {
        cout << "Could not open or find the image: PIC1_L.png" << "\n" << endl;
        return -1;
    }
    std::vector<cv::Mat> pyramid = MS_Pyramid(im_pyramid_test, 3);

    for (size_t i = 0; i < pyramid.size(); ++i) {
        cout << "Pyramid Level " << i << ": " << pyramid[i].cols << " x " << pyramid[i].rows << endl;
    }

    while(1){
        for (size_t i = 0; i < pyramid.size(); i++)
        {
            imshow("Pyramid Level " + to_string(i), pyramid[i]);
        }
        
        char c = (char)waitKey(10);
        if (c != -1) break; //Press escape to stop program
    }

    if (!fs::exists(data_dir) || !fs::is_directory(data_dir)) {
        cout << "Could not open directory: " << data_dir.string() << endl;
        return -1;
    }

    for (const auto& entry : fs::directory_iterator(data_dir)) {
        if (!entry.is_regular_file()) continue;
        string file_name = entry.path().filename().string();
        string full_path = entry.path().string();
        Mat im = imread(full_path);

        if (im.empty()) {
            cout << "Could not open or find the image: " << full_path << "\n" << endl;
            continue; // skip to next file
        }

        // Convert to grayscale
        Mat im_gray;
        cvtColor(im, im_gray, COLOR_BGR2GRAY);

        vector<char> pix_states = FAST_FEATURE_DETECTOR(im_gray, file_name);
    
    }
    while (1) {
        while (1) {
            char c = (char)waitKey(10);
            if (c != -1) break; //Press escape to stop program
        }
        break;
    }


    return 0;
}