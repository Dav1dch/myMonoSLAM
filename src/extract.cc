#include "extract.h"

namespace fe_extract {

cv::Mat pts_normalize(std::vector<cv::Point2f> pts) {
    cv::Mat mat(2, pts.size(), CV_32F, pts.data());
    cv::Mat col = cv::Mat::ones(1, mat.cols, CV_32F);

    cv::vconcat(mat, col, mat);
    std::cout << mat.size() << std::endl;

    cv::Mat K(cv::Size(3, 3), CV_32F);
    K = (cv::Mat_<float>(3, 3) << 726.21081542969, 0.0, 359.2048034668, 0.0,
         726.21081542969, 202.47247314453, 0.0, 0.0, 1.0);
    cv::Mat K_inv = K.inv();
    cv::Mat res = K_inv * mat;
    res = res.t();

    return res.t();
}

feature_extractor::feature_extractor() {
    this->last = cv::Mat();
    this->descriptors_ = cv::Mat();
    this->isInitialized = false;
    this->matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    this->matches = std::vector<cv::DMatch>();
}

void feature_extractor::extract(cv::Mat frame,
                                std::vector<cv::Point2f> &corners,
                                cv::Mat descriptors) {

    cv::Mat frame_gray, norm_pts;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    goodFeaturesToTrack(frame_gray, corners, 3000, 0.01, 7.0);
    norm_pts = pts_normalize(corners);
    std::vector<cv::KeyPoint> kps;
    for (auto c : corners) {
        kps.push_back(cv::KeyPoint(c, 20.f));
    }

    for (int i = 0; i < norm_pts.rows; ++i) {
        kps.push_back(cv::KeyPoint(norm_pts.at<float>(i, 0),
                                   norm_pts.at<float>(i, 1), 20.f));
    }

    detector->compute(frame, kps, descriptors);

    if (!this->isInitialized) {

        frame.copyTo(this->last);
        descriptors.copyTo(this->descriptors_);
        this->kps_ = std::vector<cv::KeyPoint>(kps);
        this->corners_ = std::vector<cv::Point2f>(corners);
        for (auto p : this->corners_) {
            circle(this->last, p, 3, {0, 255, 0});
        } // for loop

        cv::imshow("Frame", this->last);
        this->isInitialized = true;

    } else {

        cv::Mat vis;
        std::vector<cv::Mat> vImgs;
        for (auto p : corners) {
            circle(frame, p, 3, {0, 255, 0});
        }
        // vImgs.push_back(this->last);
        // vImgs.push_back(frame);

        matcher->match(descriptors, this->descriptors_, this->matches);

        std::vector<cv::DMatch> good_matches;
        std::vector<uchar> inliers(this->matches.size(), 0);
        std::vector<cv::Point2f> points1, points2;

        for (size_t i = 0; i < matches.size(); i++) {
            points1.push_back(kps[matches[i].queryIdx].pt);
            points2.push_back(this->kps_[matches[i].trainIdx].pt);
        }
        cv::Mat K(cv::Size(3, 3), CV_64F);
        K = (cv::Mat_<float>(3, 3) << 726.21081542969, 0.0, 359.2048034668, 0.0,
             726.21081542969, 202.47247314453, 0.0, 0.0, 1.0);

        // cv::Mat fundamental_matrix = findFundamentalMat(
        // points1, points2, cv::FM_RANSAC, 3.0, 0.9, inliers);

        cv::Mat essential_matrix = cv::findEssentialMat(
            points1, points2, K, cv::FM_RANSAC, 0.7, 3, inliers);
        cv::Mat r, t;
        cv::recoverPose(essential_matrix, points1, points2, K, r, t);
        std::cout << "r: \n" << r << std::endl;
        std::cout << "t: \n" << t << std::endl;

        for (auto i = 0; i < inliers.size(); i++) {
            if (inliers[i]) {
                good_matches.push_back(this->matches[i]);
            }
        } // for loop

        this->matches = std::vector<cv::DMatch>(good_matches);

        std::cout << "matches size: " << this->matches.size() << std::endl;
        // drawMatches(this->last, this->kps_, frame, kps,
        // this->matches, vis);
        for (auto m : this->matches) {
            line(frame, kps[m.queryIdx].pt, this->kps_[m.trainIdx].pt,
                 {255, 0, 0}, 4);
        }

        // vconcat(vImgs, vis);
        // drawMatches(this->last, this->kps_, frame,kps, matches,
        // vis);
        cv::imshow("Frame", frame);
        cv::waitKey(0);

        frame.copyTo(this->last);
        descriptors.copyTo(this->descriptors_);
        this->kps_ = std::vector<cv::KeyPoint>(kps);
        this->corners_ = std::vector<cv::Point2f>(corners);
    }
}

} // namespace fe_extract
