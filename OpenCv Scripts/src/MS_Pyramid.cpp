#include "MS_Pyramid.h"
#include <iostream>
using namespace std;

std::vector<cv::Mat> MS_Pyramid(cv::Mat img, const int& levels){
    cout << "Creating pyramid with " << levels << " levels." << endl;
    std::vector<cv::Mat> Pyramid;
    Pyramid.push_back(img);

    for(int i = 1; i<levels; i++){
        cv::Mat down = cv::Mat::zeros(Pyramid[i-1].rows/2, Pyramid[i-1].cols/2, Pyramid[i-1].type());

        for (int j=0; j<Pyramid[i-1].rows; j = j + 2){
            for (int k=0; k<Pyramid[i-1].cols; k = k + 2){
                down.at<uchar>(j/2, k/2) = Pyramid[i-1].at<uchar>(j,k);
            }
        }
        Pyramid.push_back(down);
    }
    return Pyramid;
}