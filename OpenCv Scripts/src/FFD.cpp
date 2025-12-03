#include "FFD.h"
#include <iostream>
using namespace std;

vector<Key_Point> FAST_FEATURE_DETECTOR(Mat& img, const std::string& file_name) {   
    int64 FFD_start = cv::getTickCount();       // Take start time to calculate processing time later

    cv::copyMakeBorder(img, img, 3, 3, 3, 3, BORDER_REPLICATE); // Use openCV function to add border (mirrored to start)
    cv::Mat Bres_Circ = cv::Mat::zeros(1, 16, CV_64F);  // 1x16 Vector to hold Bresenham circle points
    // vector<Point> corner_Locations; // Vector to hold corner locations
    // vector<int> scores; // Vector to hold corner scores
    vector<Key_Point> keypoints; // Vector to hold Key_Point structs
    // std::vector<std::vector<std::vector<char>>> pix_states(
    //     img.rows, std::vector<std::vector<char>>(img.cols, std::vector<char>(16, 0))
    // );    // Stores whether each of the interested pixels is brighter, darker, or the same as the center pixel.

    int I; // Center pixel intensity
    int m = 0; // Number of pixels brighter/darker than threshold
    int mt = 2; // Temporary counter for pixels greater/less than threshold
    int n = 9; // Number of required contiguous pixels for corner 
    int c = 0;  // Counter for contiguous pixels
    int c_start = 0; // Starting index for contiguous counter
    int c_max = 0; // Maximum contiguous counter
    int t = 50; // Intensity threshold
    int score = 0; // Corner score
    char last_state = 's'; // Last state of pixel (brighter, darker, same)

    // Placeholder for FAST feature detector implementation
    cout << "FAST Feature Detector function called." << endl;
    for (int i = 3; i < img.rows-3; ++i) {
        for (int j = 3; j < img.cols-3; ++j) {
            Key_Point curr_point;
            m = 0;                      // Reset counter for pixels greater/less than threshold
            c = 0;                      // Reset contiguous counter
            score = 255;                // Reset score
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
            if (m >= mt){
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
                        m++;
                        // pix_states[i][j][k] = 'b'; // Mark as brighter
                        if (c_start ==0){
                            c_start = k; // Set starting index for contiguous counter
                        }
                        if (last_state == 'd') {
                                c++;
                        } else {
                            c = 1; // Reset contiguous counter if last state was 'b'
                        }
                        if (c > c_max) {
                            c_max = c; // Update maximum contiguous counter
                        }
                        last_state = 'b'; // Update last state
                    } else if (Bres_Circ.at<double>(0, k) < I - t) {
                        // pix_states[i][j][k] = 'd'; // Mark as darker
                        m++;
                        if (c_start ==0){
                            c_start = k; // Set starting index for contiguous counter
                        }
                        if (last_state == 'd') {
                                c++;
                        } else {
                            c = 1; // Reset contiguous counter if last state was 'b'
                        }
                        if (c > c_max) {
                            c_max = c; // Update maximum contiguous counter
                        }
                        last_state = 'd'; // Update last state
                    } else {
                        // pix_states[i][j][k] = 's'; // Mark as same
                        c = 0; // Reset contiguous counter
                        c_start = 0; // Reset starting index
                        last_state = 's'; // Update last state
                    }
                }

                if (c_start != 0){ // Check for wrap-around case
                    for (int k = 0; k < c_start; k++) {
                        if (Bres_Circ.at<double>(0, k) > I + t) {
                            if (last_state == 'b') {
                                c++;
                            } else {
                                c = 1; // Reset contiguous counter if last state was 'd'
                            }
                            if (c > c_max) {
                                c_max = c; // Update maximum contiguous counter
                            }
                            last_state = 'b'; // Update last state
                        } else if (Bres_Circ.at<double>(0, k) < I - t) {
                            if (last_state == 'd') {
                                c++;
                            } else {
                                c = 1; // Reset contiguous counter if last state was 'b'
                            }
                            if (c > c_max) {
                                c_max = c; // Update maximum contiguous counter
                            }
                            last_state = 'd'; // Update last state
                        } else {
                            break; // Stop if a non-contiguous pixel is found
                        }
                    }
                }


                if (c_max >= n){
                    // Calculate the 'score' of the corner. --> Minimum absolute difference between center pixel and contiguous pixels
                    for (int k = c_start; k < c_start + c_max; k++) {
                        int diff = abs(Bres_Circ.at<double>(0, k % 16) - I);
                        if (diff < score) {
                            score = diff; // Update corner score
                        }
                    }
                    // cout << "Corner score: " << score << endl;
                    curr_point.pt = Point(j-3, i-3); // Adjust for border offset
                    curr_point.score = score;
                    keypoints.push_back(curr_point);
                }
            }        
        }
    }

    img = img(cv::Rect(3, 3, img.cols - 6, img.rows - 6)); // Remove border added at start

    int64 FFD_end = cv::getTickCount(); // Take finish time to calculate processing time 
    double FFD_time = (FFD_end - FFD_start) / cv::getTickFrequency();   
    cout << "Displayed corners detected by FAST." << endl;
    cout << "FAST Feature Detection Time: " << FFD_time << " seconds" << endl;

    ////////////////////////////////////////////
    // DISPLAY IMAGE WITH CORNERS HIGHLIGHTED //
    ////////////////////////////////////////////

    Mat coloured_img;
    cvtColor(img, coloured_img, COLOR_GRAY2BGR); // Convert to BGR for colored drawing
    for (const auto& kp : keypoints) {   // Draw red circles at corner locations
        circle(coloured_img, Point(kp.pt.x, kp.pt.y), 1, Scalar(0, 0, 255), -1); // Draw red circles at corner locations
    }
    
    string parameters = "_mt" + to_string(mt) + "_t" + to_string(t) + "_n" + to_string(n);
    string full_path = "C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\FAST Pyramid\\" + file_name + parameters + ".png";
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
    ////////////////////////////////////////////

    // If no corners were found, return an empty vector
    if (keypoints.empty()) {
        return vector<Key_Point>();
    }
    // Return the 16-element state vector for the first detected corner.
    // pix_states is indexed as [row][col] (i = y, j = x), while Point stores (x,y).
    return keypoints;
}