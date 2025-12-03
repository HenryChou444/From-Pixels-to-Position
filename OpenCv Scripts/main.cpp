#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp> 
#include "FFD.h"
#include "MS_Pyramid.h"
#include "Gaussian_Kernel_Gen.h"
#include "BRIEF.h"
#include "SIFT.h"
#include "ORB.h"
using namespace cv;
using namespace std;



int main()
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR); // Suppress info/warning logs

    namespace fs = std::filesystem;
    const fs::path data_dir = R"(C:\Users\Alec\Documents\DTU\E25\Image Analysis with Microcomputer\Final Project\From-Pixels-to-Position\Data\0_to_14_cm_v50mms\Image_data)";

    // Load in test image for pyramid generation
    Mat im_pyramid_test = imread(R"(C:\Users\Alec\Documents\DTU\E25\Image Analysis with Microcomputer\Exercises\Image-Analysis-with-Microcomputer-Exercises\frame_00013.png)", IMREAD_GRAYSCALE);
    if (im_pyramid_test.empty()) {
        cout << "Could not open or find the image: frame_00013.png" << "\n" << endl;
        return -1;
    }

    int N = 1000; // Number of keypoints to retain from FAST
    std::vector<Key_Point> all_keypoints;

    int run_pyramid = 0; // Set to 1 to run pyramid FAST detection
    if (run_pyramid == 1){
        int L = 3; // Number of pyramid levels

        // Generate the image pyramid
        std::vector<cv::Mat> pyramid = MS_Pyramid(im_pyramid_test, L);

        // Run FAST algorithm on each layer of the pyramid 
        for (size_t i = 0; i < pyramid.size(); ++i) {
            cout << "Pyramid Level " << i << ": " << pyramid[i].cols << " x " << pyramid[i].rows << endl;
            string file_name = "Pyramid_Level_" + to_string(i) + ".png";
            vector<Key_Point> keypoints = FAST_FEATURE_DETECTOR(pyramid[i], file_name);
            if (i == 0){
                for (auto kp : keypoints){
                    all_keypoints.push_back(kp); // keep score and point
                }
            }
            if (i > 0){ 
                cout << "Scaled keypoints by factor of " << pow(2, i) << " to original image size." << endl;
                for (auto& kp : keypoints) {
                    kp.pt.x *= pow(2, i);
                    kp.pt.y *= pow(2, i);
                    all_keypoints.push_back(kp);
                }
            }
            cout << "Detected " << keypoints.size() << " keypoints at Pyramid Level " << i << endl;
        }
        vector<Key_Point> keypoints_test = all_keypoints;
    }

    Mat FFD_Test = imread(R"(C:\Users\Alec\Documents\DTU\E25\Image Analysis with Microcomputer\Exercises\Image-Analysis-with-Microcomputer-Exercises\frame_00013.png)", IMREAD_GRAYSCALE);
    vector<Key_Point> keypoints_test = FAST_FEATURE_DETECTOR(FFD_Test, "Test_Image");
    cout << "Detected " << keypoints_test.size() << " keypoints in test image." << endl;

    cout << "Keypoint 1 score: " << keypoints_test[0].score << endl;
    std::sort(keypoints_test.begin(), keypoints_test.end(), [](const Key_Point& a, const Key_Point& b) {
        return a.score > b.score;
    });

    int r = 25;
    int cell_size = r; // r - radius to search around FAST features
    std::map<std::pair<int, int>, std::vector<Key_Point>> grid; // Break image into a grid with cell size == r
    std::map<std::pair<int, int>, int> erase_points; // Points to erase due to higher score in neighboring cell

    for (const auto& kp : keypoints_test) { // Go through all keypoints and add them to their cell on the grid 
        int cell_x = kp.pt.x / cell_size;
        int cell_y = kp.pt.y / cell_size;
        grid[{cell_x, cell_y}].push_back(kp);
    }

    for (const auto& kp : keypoints_test) {
        int cell_x = kp.pt.x / cell_size;
        int cell_y = kp.pt.y / cell_size;

        //cout << "erase_points at: " << kp.pt.x << ", " << kp.pt.y << " == " << (erase_points.find({kp.pt.x, kp.pt.y}) != erase_points.end()) << endl;

        if (erase_points.find({kp.pt.x, kp.pt.y}) != erase_points.end()) {
            // cout << "Skipping keypoint at (" << kp.pt.x << ", " << kp.pt.y << ") already marked for erasure." << endl;
            continue; // Already marked for erasure
        }

        for (int i = -1; i <= 1; ++i) { // Search all neighboring cells for keypoints
            for (int j = -1; j <= 1; ++j) {
                int neighbor_x = cell_x + i;
                int neighbor_y = cell_y + j;
                auto it = grid.find({neighbor_x, neighbor_y});
                if (it != grid.end()) {
                    //cout << it->first.second << "Cell exists with " << it->second[0].pt << " keypoint." << it->first.first << endl;
                    for (int k = 0; k < it->second.size(); ++k) {
                        const Key_Point& neighbor_kp = it->second[k];
                        if (neighbor_kp.pt.x == kp.pt.x && neighbor_kp.pt.y == kp.pt.y) continue; // Skip self-comparison
                        double dist = sqrt(pow(neighbor_kp.pt.x - kp.pt.x, 2) + pow(neighbor_kp.pt.y - kp.pt.y, 2));
                        if (dist <= r) {
                            if (neighbor_kp.score >= kp.score) {
                                erase_points[{kp.pt.x, kp.pt.y}] = 1; // Mark for erasure
                            }
                            else {
                                erase_points[{neighbor_kp.pt.x, neighbor_kp.pt.y}] = 1; // Mark neighbor for erasure
                            }
                        }
                    }
                }
            }
        }
        
        // You can use count_in_cell for further processing if needed
    }

    cout << "Points to erase due to neighboring higher score: " << erase_points.size() << endl;

    keypoints_test.erase(
        std::remove_if(keypoints_test.begin(), keypoints_test.end(),
            [&](const Key_Point& kp) {
                return erase_points.count({kp.pt.x, kp.pt.y});
            }),
        keypoints_test.end()
    );

    ////////////////////////////////////////
    //  Non-maximum suppression complete  //
    ////////////////////////////////////////

    cout << " Number of keypoints before limiting: " << keypoints_test.size() << endl;
    if (keypoints_test.size() > N) {
        keypoints_test.erase(keypoints_test.begin() + N, keypoints_test.end());
    }

    std::vector<std::vector<int>> descriptors = BRIEF_descriptors(FFD_Test, keypoints_test, 128, 31); // 256-bit descriptors, patch size 31x31
    cout << "First descriptor values:" << descriptors[0][0] << ", " << descriptors[0][1] << ", " << descriptors[0][2] << ", ..." << endl;


    cout << "Top 10 keypoint scores after sorting and limiting to N:" << endl;
    for (size_t i = 0; i < min(size_t(10), keypoints_test.size()); ++i) {
        cout << keypoints_test[i].score << " ";
    }
    cout << endl;

    Mat coloured_img;
    cvtColor(FFD_Test, coloured_img, COLOR_GRAY2BGR); // Convert to BGR for colored drawing
    for (const auto& kp : keypoints_test) {   // Draw red circles at corner locations
        circle(coloured_img, Point(kp.pt.x, kp.pt.y), r/2, Scalar(0, 0, 255), 1); // Draw red circles at corner locations (Using r/2 means that circles should not overlap)
        // Draw grid of 25x25 cells
        // int cell_x = (kp.pt.x / cell_size) * cell_size;
        // int cell_y = (kp.pt.y / cell_size) * cell_size;
        // rectangle(coloured_img, Point(cell_x, cell_y), Point(cell_x + cell_size, cell_y + cell_size), Scalar(255, 0, 0), 1);
        
    }
    string full_path = "C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\FAST Pyramid\\frame_00013.png";
    cout << "Saving FAST features to: " << full_path << endl;
    imwrite(full_path, coloured_img); // Save the output image
    

    while (1) {
        imshow("Corners", coloured_img);
        // cap >> output;
        // filter2D(output, output, -1, kernel_gl);
        // imshow("webcam input", output);
        char c = (char)waitKey(10);
        if (c != -1) break; //Press escape to stop program
        // Save image
        imwrite("C:\\Users\\Alec\\Documents\\DTU\\E25\\Image Analysis with Microcomputer\\Exercises\\corners.png", coloured_img);
    }

    // Display pyramid layers 
    // while(1){
    //     for (size_t i = 0; i < pyramid.size(); i++)
    //     {
    //         imshow("Pyramid Level " + to_string(i), pyramid[i]);
    //     }
        
    //     char c = (char)waitKey(10);
    //     if (c != -1) break; //Press escape to stop program
    // }

    if (!fs::exists(data_dir) || !fs::is_directory(data_dir)) {
        cout << "Could not open directory: " << data_dir.string() << endl;
        return -1;
    }

    //////////////////////////////////////////////////////
    // Iterate through each image file in the directory //
    //////////////////////////////////////////////////////
    // for (const auto& entry : fs::directory_iterator(data_dir)) {
    //     if (!entry.is_regular_file()) continue;
    //     string file_name = entry.path().filename().string();
    //     string full_path = entry.path().string();
    //     Mat im = imread(full_path);

    //     if (im.empty()) {
    //         cout << "Could not open or find the image: " << full_path << "\n" << endl;
    //         continue; // skip to next file
    //     }

    //     // Convert to grayscale
    //     Mat im_gray;
    //     cvtColor(im, im_gray, COLOR_BGR2GRAY);

    //     vector<Key_Point> keypoints = FAST_FEATURE_DETECTOR(im_gray, file_name);
    // }


    while (1) {
        while (1) {
            char c = (char)waitKey(10);
            if (c != -1) break; //Press escape to stop program
        }
        break;
    }


    return 0;
}