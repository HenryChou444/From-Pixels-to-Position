#include "BRIEF_Comparison.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>
#include <random>
using namespace std;

std::vector<std::vector<int>> BRIEF_Comparison(const std::vector<Key_Point>& keypoints1, const std::vector<Key_Point>& keypoints2, const std::vector<std::vector<int>>& descriptors1, const std::vector<std::vector<int>>& descriptors2) { 

    std::map<int, int> matches; // Map to hold matches: key = index in descriptors1, value = index in descriptors2
    std::map<int, int> already_matched; // To keep track of already matched descriptors in descriptors2

    for (int i = 0; i < descriptors1.size(); i++){
        // int best_match_index = -1;
        // int best_match_distance = INT_MAX;
        // for (int j = 0; j < descriptors2.size(); j++){
        //     int distance = 0;
        //     int desc_XOR = descriptors1[i] ^ descriptors2[j];
        //     distance = std::popcount(desc_XOR);
        //     if ((distance < best_match_distance) && (already_matched.find(j) == already_matched.end())){
        //         best_match_distance = distance;
        //         best_match_index = j;
        //         matches[i] = best_match_index;
        //         already_matched[best_match_index] = 1;
        //     }
        // }
    }
    
}