#include "FFD.h"
#include <iostream>
using namespace std;

vector<char> FAST_FEATURE_DETECTOR(Mat& img, const std::string& file_name) {   
    int64 FFD_start = cv::getTickCount();       // Take start time to calculate processing time later

    cv::copyMakeBorder(img, img, 3, 3, 3, 3, BORDER_REPLICATE); // Use openCV function to add border (mirrored to start)
    cv::Mat Bres_Circ = cv::Mat::zeros(1, 16, CV_64F);  // 1x16 Vector to hold Bresenham circle points
    vector<Point> corner_Locations; // Vector to hold corner locations
    std::vector<std::vector<std::vector<char>>> pix_states(
        img.rows, std::vector<std::vector<char>>(img.cols, std::vector<char>(16, 0))
    );    // Stores whether each of the interested pixels is brighter, darker, or the same as the center pixel.

    int I; // Center pixel intensity
    int m = 0; // Number of pixels brighter/darker than threshold
    int n = 12; // Number of required contiguous pixels for corner 
    int c = 0;  // Counter for contiguous pixels
    int c_max = 0; // Maximum contiguous counter
    int t = 30; // Intensity threshold

    // Placeholder for FAST feature detector implementation
    cout << "FAST Feature Detector function called." << endl;
    for (int i = 3; i < img.rows-3; ++i) {
        for (int j = 3; j < img.cols-3; ++j) {
            m = 0;                      // Reset counter for pixels greater/less than threshold
            c = 0;                      // Reset contiguous counter
            I = img.at<uchar>(i, j);    // Center pixel intensity

            Bres_Circ.setTo(I); // Initialize all points to the center pixel value
            Bres_Circ.at<double>(0, 0) = img.at<uchar>(i-3, j); // Find intensity of pixels 1, 5, 9, 13 
            Bres_Circ.at<double>(0, 4) = img.at<uchar>(i, j+3);
            Bres_Circ.at<double>(0, 8) = img.at<uchar>(i+3, j);
            Bres_Circ.at<double>(0, 12) = img.at<uchar>(i, j-3);
            for (int k = 0; k<16; k = k+4){ // Check how many of pixels 1, 5, 9, 13 are greater than I+t or less than I-t
                if (Bres_Circ.at<double>(0, k) > I + t || Bres_Circ.at<double>(0, k) < I - t){
                    m = m + 1;
                }
            }
            if (m >= 3){
                Bres_Circ.at<double>(0, 1) = img.at<uchar>(i-3, j+1);
                Bres_Circ.at<double>(0, 2) = img.at<uchar>(i-2, j+2);
                Bres_Circ.at<double>(0, 3) = img.at<uchar>(i-1, j+3);
                Bres_Circ.at<double>(0, 5) = img.at<uchar>(i-1, j+3);
                Bres_Circ.at<double>(0, 6) = img.at<uchar>(i-2, j+2);
                Bres_Circ.at<double>(0, 7) = img.at<uchar>(i-3, j+1);
                Bres_Circ.at<double>(0, 9) = img.at<uchar>(i+1, j+3);
                Bres_Circ.at<double>(0, 10) = img.at<uchar>(i+2, j+2);
                Bres_Circ.at<double>(0, 11) = img.at<uchar>(i+3, j+1);
                Bres_Circ.at<double>(0, 13) = img.at<uchar>(i+3, j-1);
                Bres_Circ.at<double>(0, 14) = img.at<uchar>(i+2, j-2);
                Bres_Circ.at<double>(0, 15) = img.at<uchar>(i+1, j-3);
                
                for (int k = 0; k < 16; k++) {
                    if (Bres_Circ.at<double>(0, k) > I + t) {
                        c++;
                        m++;
                        pix_states[i][j][k] = 'b'; // Mark as brighter
                        if (c > c_max) {
                            c_max = c; // Update maximum contiguous counter
                        }
                    } else if (Bres_Circ.at<double>(0, k) < I - t) {
                        c++;
                        pix_states[i][j][k] = 'd'; // Mark as darker
                        m++;
                        if (c > c_max) {
                            c_max = c; // Update maximum contiguous counter
                        }
                    } else {
                        pix_states[i][j][k] = 's'; // Mark as same
                        c = 0; // Reset contiguous counter
                    }
                }

                if (c_max >= n){
                    // Mark as corner - for example, set to white
                    corner_Locations.push_back(Point(j, i));
                }
            }        
        }
    }

    img = img(cv::Rect(3, 3, img.cols - 6, img.rows - 6)); // Remove border added at start

    int64 FFD_end = cv::getTickCount(); // Take finish time to calculate processing time 
    double FFD_time = (FFD_end - FFD_start) / cv::getTickFrequency();   
    cout << "Displayed corners detected by FAST." << endl;
    cout << "FAST Feature Detection Time: " << FFD_time << " seconds" << endl;

    Mat coloured_img;
    cvtColor(img, coloured_img, COLOR_GRAY2BGR); // Convert to BGR for colored drawing
    for (const auto& pt : corner_Locations) {   // Draw red circles at corner locations
        circle(coloured_img, pt, 1, Scalar(0, 0, 255), -1); // Draw red circles at corner locations
    }
    string full_path = "C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\FAST Results\\" + file_name;
    cout << "Saving FAST features to: " << full_path << endl;
    imwrite(full_path, coloured_img); // Save the output image
    

    // while (1) {
    //     imshow("Corners", coloured_img);
    //     // cap >> output;
    //     // filter2D(output, output, -1, kernel_gl);
    //     // imshow("webcam input", output);
    //     char c = (char)waitKey(10);
    //     if (c != -1) break; //Press escape to stop program
    //     // Save image
    //     imwrite("C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\corners.png", coloured_img);
    // }
    // If no corners were found, return an empty vector
    if (corner_Locations.empty()) {
        return std::vector<char>();
    }
    // Return the 16-element state vector for the first detected corner.
    // pix_states is indexed as [row][col] (i = y, j = x), while Point stores (x,y).
    return pix_states.at(corner_Locations[0].y).at(corner_Locations[0].x);
}