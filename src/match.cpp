#include "match.h"
#include <_types/_uint8_t.h>

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return cv::Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                       (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

feature_matcher::feature_matcher() {
    // this->K = (cv::Mat_<double>(3, 3) << 726.21081542969, 0.0, 359.2048034668,
    //            0.0, 726.21081542969, 202.47247314453, 0.0, 0.0, 1.0);
    this->K = (cv::Mat_<double>(3, 3) << 517.3, 0.0, 318.6, 0.0, 516.5, 255.3,
               0.0, 0.0, 1.0);
    // this->K =
    //     (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    // this->K = (cv::Mat_<double>(3, 3) << 270.0, 0.0, 960.0, 0.0, 270.0, 540.0,
    //            0.0, 0.0, 1.0);
    this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    this->matches = std::vector<cv::DMatch>();
}

void feature_matcher::match_frames(Frame *f1, Frame *f2) {

    std::vector<std::vector<cv::DMatch>> matches;
    // this->matcher->knnMatch(f2->descriptors, f1->descriptors, this->matches,
    // 2);
    this->matcher->knnMatch(f2->descriptors, f1->descriptors, matches, 2);

    // this->matcher->match(f2->descriptors, f1->descriptors, this->matches);
    std::set<int> idx1s, idx2s;
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2d> points1, points2;

    for (auto mVec : matches) {
        if (mVec[0].distance < 0.75 * mVec[1].distance) {
            if (mVec[0].distance < 32) {
                if (idx2s.find(mVec[0].queryIdx) == idx2s.end() &&
                    idx1s.find(mVec[0].trainIdx) == idx1s.end()) {
                    idx1s.insert(mVec[0].trainIdx);
                    idx2s.insert(mVec[0].queryIdx);
                    good_matches.push_back(mVec[0]);
                    points1.push_back(f1->kps_all[mVec[0].trainIdx].pt);
                    points2.push_back(f2->kps_all[mVec[0].queryIdx].pt);
                }
            }
        }
    }

    // auto minmax =
    //     minmax_element(this->matches.begin(), this->matches.end(),
    //                    [](const cv::DMatch &m1, const cv::DMatch &m2) {
    //                        return m1.distance < m2.distance;
    //                    });
    // double min_dist = minmax.first->distance;
    // double max_dist = minmax.second->distance;

    std::vector<uchar> inliers(matches.size(), 0);

    // for (int i = 0; i < this->matches.size(); i++) {
    //     if (this->matches[i].distance <= std::max(2 * min_dist, 30.0)) {
    //         good_matches.push_back(this->matches[i]);
    //         points1.push_back(f1->kps_all[this->matches[i].trainIdx].pt);
    //         points2.push_back(f2->kps_all[this->matches[i].queryIdx].pt);
    //     }
    // }
    // std::cout << "min_dist " << min_dist << std::endl;
    // std::cout << "max_dist " << max_dist << std::endl;
    // std::cout << "size " << good_matches.size() << std::endl;
    //
    // cv::Mat m1 = pts_normalize(points1);
    // cv::Mat m2 = pts_normalize(points2);

    // cv::Mat fundamental_matrix = findFundamentalMat(
    // points1, points2, cv::FM_RANSAC, 3.0, 0.9, inliers);

    cv::Mat mask;
    f2->essential_matrix = cv::findEssentialMat(points1, points2, this->K);

    // for (auto i = 0; i < inliers.size(); i++) {
    //     if (inliers[i]) {
    //         good_matches.push_back(this->matches[i]);
    //     }
    // } // for loop

    this->matches = std::vector<cv::DMatch>(good_matches);
    for (auto i = 0; i < this->matches.size(); i++) {

        f1->pts2.push_back(f1->kps_all[this->matches[i].trainIdx].pt);
        f2->pts1.push_back(f2->kps_all[this->matches[i].queryIdx].pt);
        f2->match_idxs[this->matches[i].queryIdx] = i;

        f1->points2.push_back(
            pixel2cam(f1->kps_all[this->matches[i].trainIdx].pt, this->K));
        f2->points1.push_back(
            pixel2cam(f2->kps_all[this->matches[i].queryIdx].pt, this->K));
    }
    // std::cout << m1.row(this->matches[0].trainIdx) << std::endl;
}
