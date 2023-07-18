#include "extract.h"

namespace fe_extract {

feature_extractor::feature_extractor() {
    this->last = Mat();
    this->descriptors_ = Mat();
    this->isInitialized = false;
    this->matcher = BFMatcher::create(NORM_HAMMING, true);
    this->matches = vector<DMatch>();
}

void feature_extractor::extract(Mat frame, vector<Point2f> &corners,
                                Mat descriptors) {

    Mat frame_gray;
    Ptr<ORB> detector = ORB::create();
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(frame_gray, corners, 3000, 0.01, 7.0);
    vector<KeyPoint> kps;
    for (auto c : corners) {
        kps.push_back(KeyPoint(c, 20.f));
    }

    detector->compute(frame, kps, descriptors);

    if (!this->isInitialized) {

        frame.copyTo(this->last);
        descriptors.copyTo(this->descriptors_);
        this->kps_ = vector<KeyPoint>(kps);
        this->corners_ = vector<Point2f>(corners);
        for (auto p : this->corners_) {
            circle(this->last, p, 3, {0, 255, 0});
        }
        imshow("Frame", this->last);
        this->isInitialized = true;

    } else {

        Mat vis;
        vector<Mat> vImgs;
        for (auto p : corners) {
            circle(frame, p, 3, {0, 255, 0});
        }
        vImgs.push_back(this->last);
        vImgs.push_back(frame);

        matcher->match(descriptors, this->descriptors_, this->matches);

        // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
        std::vector<DMatch> good_matches;
        vector<uchar> inliers(this->matches.size(), 0);
        std::vector<cv::Point2f> points1, points2;

        for (size_t i = 0; i < matches.size(); i++) {
            points1.push_back(kps[matches[i].queryIdx].pt);
            points2.push_back(this->kps_[matches[i].trainIdx].pt);
        }

        Mat fundamental_matrix = findFundamentalMat(
            points1, points2, cv::FM_RANSAC, 3.0, 0.9, inliers);

        for (auto i = 0; i < inliers.size(); i++) {
            if (inliers[i]) {
                good_matches.push_back(this->matches[i]);
            }
        }
        this->matches = vector<DMatch>(good_matches);

        cout << this->matches.size() << endl;
        // drawMatches(this->last, this->kps_, frame, kps, this->matches, vis);
        for (auto m : this->matches) {
            line(frame, kps[m.queryIdx].pt, this->kps_[m.trainIdx].pt,
                 {255, 0, 0}, 4);
        }

        // vconcat(vImgs, vis);
        // drawMatches(this->last, this->kps_, frame,kps, matches, vis);
        imshow("Frame", frame);
        cv::waitKey(0);

        frame.copyTo(this->last);
        descriptors.copyTo(this->descriptors_);
        this->kps_ = vector<KeyPoint>(kps);
        this->corners_ = vector<Point2f>(corners);
    }
}

} // namespace fe_extract
