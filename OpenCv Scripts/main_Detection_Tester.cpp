#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp> 
#include "FFD.h"
#include "Gaussian_Kernel_Gen.h"
#include "SIFT.h"
#include "ORB.h"
using namespace cv;
using namespace std;



int main()
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR); // Suppress info/warning logs

    // Load photo from file - change path as needed
    Mat im = imread("C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\PIC1_L.png");

    if (im.empty()) {
        cout << "Could not open or find the image!\n" << endl;
        return -1;
    }

    // Convert to grayscale
    Mat im_gray;
    cvtColor(im, im_gray, COLOR_BGR2GRAY);

    Mat corner_img;
    vector<char> pix_states = FAST_FEATURE_DETECTOR(im_gray);
    cout << "Pixel states around first detected corner: ";
    for (char state : pix_states) {
        cout << state << " ";
    }

    Mat sift_features;
    sift_features = SIFT_DETECTOR(im_gray);

    Mat orb_features;
    orb_features = ORB_DETECTOR(im_gray);

    while (1) {


        while (1) {
            // cap >> output;
            // filter2D(output, output, -1, kernel_gl);
            // imshow("webcam input", output);
            char c = (char)waitKey(10);
            if (c != -1) break; //Press escape to stop program
        }
        break;
    }


    return 0;
}