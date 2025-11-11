#include "Gaussian_Kernel_Gen.h"
#include <iostream>
using namespace std;

Mat Gaussian_Kernel(int size, double sigma){
    Mat kernel(1, size, CV_64F);
    double cumm_sum = 0;

    for (int i = 0; i<size; i++){
        kernel.at<double>(0,i) = (1.0/(sigma*sqrt(2*CV_PI)))*exp(-0.5*pow((i-size/2)/sigma,2));
        cumm_sum += kernel.at<double>(0,i);
        cout << (1/(sigma*sqrt(2*CV_PI)))*exp(-0.5*pow((i-size/2)/sigma,2)) << endl;
    }

    kernel /= cumm_sum; // Normalize the kernel

    cout << "Gaussian Kernel: [";
    for (size_t i = 0; i < kernel.cols; ++i) {
        cout << kernel.at<double>(0,i);
        if (i != kernel.cols - 1) cout << ", ";
    }
    cout << "]" << endl;
    return kernel;
}