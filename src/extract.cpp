#include "extract.h"

feature_extractor::feature_extractor() {

    this->last = cv::Mat();
    this->descriptors_ = cv::Mat();
    this->isInitialized = false;

    this->detector = cv::ORB::create();
}

void feature_extractor::extract(cv::Mat image, Frame *f) {

    f->image = image;
    image.copyTo(f->image);
    cv::Mat frame_gray, norm_pts;
    cvtColor(f->image, frame_gray, cv::COLOR_BGR2GRAY);
    goodFeaturesToTrack(frame_gray, f->corners, 1600, 0.01, 5.0);
    // norm_pts = pts_normalize(f->corners);
    for (auto c : f->corners) {
        f->kps_all.push_back(cv::KeyPoint(c, 20.f));
    }
    // this->detector->detect(f->image, f->kps_all);

    // for (int i = 0; i < norm_pts.rows; ++i) {
    // f->kps.push_back(cv::KeyPoint(norm_pts.at<double>(i, 0),
    // norm_pts.at<double>(i, 1), 2.f));
    // }

    this->detector->compute(f->image, f->kps_all, f->descriptors);
    f->point_idxs = std::vector<int>(f->kps_all.size(), -1);
    f->match_idxs = std::vector<int>(f->kps_all.size(), -1);
}
