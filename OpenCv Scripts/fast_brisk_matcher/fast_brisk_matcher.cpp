#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#ifdef _WIN32
#include <windows.h>
#include <algorithm>
#include <cctype>
#endif



void Fast_Brisk_matcher_test() {
    // Input images, change with relevant path on your system

	// Artificial Rotation
	Mat img1 = imread("C:\\Images_30330\\Feature_Matching_artificial_rotation\\Image_00083.png", IMREAD_GRAYSCALE);
	Mat img2 = imread("C:\\Images_30330\\Feature_Matching_artificial_rotation\\Image_00083_rotated_40deg.png", IMREAD_GRAYSCALE);

	// Artificial Scaling
	//Mat img1 = imread("C:\\Images_30330\\Feature_Matching_artificial_scaling\\Image_00083.png", IMREAD_GRAYSCALE);
	//Mat img2 = imread("C:\\Images_30330\\Feature_Matching_artificial_scaling\\Image_00083_scale_0.5.png", IMREAD_GRAYSCALE);

	// Regular dataset images
	//Mat img1 = imread("C:\\Images_30330\\Features_Matching\\Image_00133.png", IMREAD_GRAYSCALE);
	//Mat img2 = imread("C:\\Images_30330\\Features_Matching\\Image_00134.png", IMREAD_GRAYSCALE);

	// Check if images loaded successfully
	if (img1.empty()) {
		cout << "Error: Could not load Image_00083.png" << endl;
		return;
	}
	if (img2.empty()) {
		cout << "Error: Could not load Image_00083_rotated_40deg.png" << endl;
		return;
	}
	cout << "Images loaded successfully - Image1: " << img1.size() << ", Image2: " << img2.size() << endl;
    // Variables to hold keypoints and descriptors
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	
    // Initialize Fast detector
    int threshold = 30;
	Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(threshold, true, FastFeatureDetector::TYPE_9_16);
	
    // Initialize BRISK descriptor
	int thresh = 30;              // AGAST detection threshold score (default: 30)
	//int octaves = 3;              // Detection octaves. Use 0 to do single scale (default: 3)
	int octaves = 0;
	float patternScale = 1.0f;    // Scale applied to pattern used for sampling (default: 1.0f)
	// patternScale affects descriptors only 
	Ptr<BRISK> descriptor = BRISK::create(thresh, octaves, patternScale);

	// === DETECTION PHASE ===
	cout << "\n=== DETECTION PHASE (FAST) ===" << endl;

	// Detection timing for Image 1
	int64 start_detect1 = getTickCount();
	detector->detect(img1, keypoints1);
	int64 end_detect1 = getTickCount();
	double time_detect1 = (end_detect1 - start_detect1) / getTickFrequency();

	// Detection timing for Image 2
	int64 start_detect2 = getTickCount();
	detector->detect(img2, keypoints2);
	int64 end_detect2 = getTickCount();
	double time_detect2 = (end_detect2 - start_detect2) / getTickFrequency();

	// Detection results
	cout << "Image 1 - Detection Time: " << time_detect1 * 1000 << " ms, Keypoints: " << keypoints1.size() << endl;
	cout << "Image 2 - Detection Time: " << time_detect2 * 1000 << " ms, Keypoints: " << keypoints2.size() << endl;
	double total_detection_time = time_detect1 + time_detect2;
	cout << "Total Detection Time: " << total_detection_time * 1000 << " ms" << endl;

	// === DESCRIPTION PHASE ===
	cout << "\n=== DESCRIPTION PHASE (BRISK) ===" << endl;

	// Description timing for Image 1
	int64 start_desc1 = getTickCount();
	descriptor->compute(img1, keypoints1, descriptors1);
	int64 end_desc1 = getTickCount();
	double time_desc1 = (end_desc1 - start_desc1) / getTickFrequency();

	// Description timing for Image 2
	int64 start_desc2 = getTickCount();
	descriptor->compute(img2, keypoints2, descriptors2);
	int64 end_desc2 = getTickCount();
	double time_desc2 = (end_desc2 - start_desc2) / getTickFrequency();

	// Description results
	cout << "Image 1 - Description Time: " << time_desc1 * 1000 << " ms, Descriptors: " << descriptors1.size() << endl;
	cout << "Image 2 - Description Time: " << time_desc2 * 1000 << " ms, Descriptors: " << descriptors2.size() << endl;
	double total_description_time = time_desc1 + time_desc2;
	cout << "Total Description Time: " << total_description_time * 1000 << " ms" << endl;

	// === TOTAL FEATURE EXTRACTION TIME ===
	double total_feature_time = total_detection_time + total_description_time;
	cout << "\n=== FEATURE EXTRACTION SUMMARY ===" << endl;
	cout << "Total Detection Time: " << total_detection_time * 1000 << " ms" << endl;
	cout << "Total Description Time: " << total_description_time * 1000 << " ms" << endl;
	cout << "Total Feature Extraction Time: " << total_feature_time * 1000 << " ms" << endl;
    
    
    // Initizalize Brute Force Matcher with Hamming distance and cross-check enabled
	BFMatcher matcher_cross(NORM_HAMMING, true);
	vector<DMatch> cross_matches;

	int64 start_match2 = getTickCount();
	
    try {
		matcher_cross.match(descriptors1, descriptors2, cross_matches);
	}
	catch (const cv::Exception& e) {
		cout << "Error during cross-check matching: " << e.what() << endl;
		return;
	}

	int64 end_match2 = getTickCount();
	double time_match2 = (end_match2 - start_match2) / getTickFrequency();
	cout << "\n=== MATCHING PHASE ===" << endl;
	cout << "Cross-Check Matching Time: " << time_match2 * 1000 << " ms" << endl;
	cout << "Cross-Check Matches found: " << cross_matches.size() << endl;

	// Sort and draw 20 best matches for visualization
	Mat img_matches_cross;
	if (!cross_matches.empty()) {
		sort(cross_matches.begin(), cross_matches.end(), [](const DMatch& a, const DMatch& b) {
			return a.distance < b.distance;
			});

		size_t num_matches_cross = min(static_cast<size_t>(20), cross_matches.size());
		vector<DMatch> best_matches_cross(cross_matches.begin(), cross_matches.begin() + num_matches_cross);

		cout << "Drawing " << best_matches_cross.size() << " best matches (Cross-check method)" << endl;

		//drawMatches(img1, keypoints1, img2, keypoints2, best_matches_cross, img_matches_cross,
			//Scalar(255, 0, 255), Scalar(255, 0, 255));
		drawMatches(img1, keypoints1, img2, keypoints2, best_matches_cross, img_matches_cross,
			Scalar::all(-1), Scalar::all(-1));

		namedWindow("Best Matches - Cross Check", WINDOW_NORMAL);
		imshow("Best Matches - Cross Check", img_matches_cross);
	}

	// === FINAL TIMING SUMMARY ===
	double total_pipeline_time = total_feature_time + time_match2;
	cout << "\n=== COMPLETE PIPELINE TIMING ===" << endl;
	cout << "Feature Extraction: " << total_feature_time * 1000 << " ms" << endl;
	cout << "Matching: " << time_match2 * 1000 << " ms" << endl;
	cout << "Total Pipeline Time: " << total_pipeline_time * 1000 << " ms" << endl;

	// === SAVE RESULTS ===
    //Change path as needed
	string outputPath = "C:\\Images_30330\\Feature_Matching_artificial_rotation\\";
	string baseName = "Fast_detection_Brisk_descriptor";

	// Save the displayed image
	if (!img_matches_cross.empty()) {
		string imagePath = outputPath + baseName + ".png";
		bool success = imwrite(imagePath, img_matches_cross);
		if (success) {
			cout << "\nImage saved to: " << imagePath << endl;
		}
		else {
			cout << "\nError: Failed to save image to: " << imagePath << endl;
		}
	}

	// Create and save CSV file with timing information
	string csvPath = outputPath + baseName + ".csv";
	ofstream csvFile(csvPath);
	if (csvFile.is_open()) {
		// Write CSV header
		csvFile << "Phase,Image1_Time_ms,Image2_Time_ms,Total_Time_ms,Additional_Info" << endl;

		// Write timing data
		csvFile << "Detection," << time_detect1 * 1000 << "," << time_detect2 * 1000 << ","
			<< total_detection_time * 1000 << "," << keypoints1.size() + keypoints2.size() << "_total_keypoints" << endl;

		csvFile << "Description," << time_desc1 * 1000 << "," << time_desc2 * 1000 << ","
			<< total_description_time * 1000 << "," << descriptors1.rows + descriptors2.rows << "_total_descriptors" << endl;

		csvFile << "Feature_Extraction,," << "," << total_feature_time * 1000 << ","
			<< "Detection+Description" << endl;

		csvFile << "Matching,," << "," << time_match2 * 1000 << ","
			<< cross_matches.size() << "_matches_found" << endl;

		csvFile << "Total_Pipeline,," << "," << total_pipeline_time * 1000 << ","
			<< "Complete_process" << endl;

		// Additional summary information
		csvFile << "\nSummary Information" << endl;
		csvFile << "Image1_Keypoints," << keypoints1.size() << endl;
		csvFile << "Image2_Keypoints," << keypoints2.size() << endl;
		csvFile << "Total_Matches," << cross_matches.size() << endl;
		csvFile << "Displayed_Matches," << min(static_cast<size_t>(20), cross_matches.size()) << endl;

		csvFile.close();
		cout << "CSV file saved to: " << csvPath << endl;
	}
	else {
		cout << "Error: Could not create CSV file at: " << csvPath << endl;
	}

	waitKey(0);
}