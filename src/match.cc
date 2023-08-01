#include "match.h"
#include <iostream>

namespace fe_matcher {

feature_matcher::feature_matcher() {
    this->K = (cv::Mat_<float>(3, 3) << 726.21081542969, 0.0, 359.2048034668,
               0.0, 726.21081542969, 202.47247314453, 0.0, 0.0, 1.0);
    this->matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    this->matches = std::vector<cv::DMatch>();
}

void feature_matcher::match_frames(frame_ *f1, frame_ *f2) {

    this->matcher->match(f2->descriptors, f1->descriptors, this->matches);

    std::vector<cv::DMatch> good_matches;
    std::vector<uchar> inliers(this->matches.size(), 0);
    std::vector<cv::Point2f> points1, points2;

    for (size_t i = 0; i < this->matches.size(); i++) {
        points1.push_back(f2->kps[this->matches[i].queryIdx].pt);
        points2.push_back(f1->kps[this->matches[i].trainIdx].pt);
    }

    // cv::Mat fundamental_matrix = findFundamentalMat(
    // points1, points2, cv::FM_RANSAC, 3.0, 0.9, inliers);

    f2->essential_matrix = cv::findEssentialMat(points1, points2, this->K,
                                                cv::FM_RANSAC, 0.7, 3, inliers);

    for (auto i = 0; i < inliers.size(); i++) {
        if (inliers[i]) {
            good_matches.push_back(this->matches[i]);
        }
    } // for loop

    this->matches = std::vector<cv::DMatch>(good_matches);
    for (size_t i = 0; i < this->matches.size(); i++) {
        f2->points1.push_back(f2->kps[this->matches[i].queryIdx].pt);
        f1->points2.push_back(f1->kps[this->matches[i].trainIdx].pt);
    }
}

} // namespace fe_matcher
