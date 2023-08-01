#include "extract.h"

namespace fe_extract {

feature_extractor::feature_extractor() {

    this->last = cv::Mat();
    this->descriptors_ = cv::Mat();
    this->isInitialized = false;

    this->detector = cv::ORB::create();

    this->matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    this->matches = std::vector<cv::DMatch>();
}

void feature_extractor::extract(cv::Mat image_f, frame_ *f) {

    f->image = image_f;
    cv::Mat frame_gray, norm_pts;
    cvtColor(f->image, frame_gray, cv::COLOR_BGR2GRAY);
    goodFeaturesToTrack(frame_gray, f->corners, 3000, 0.01, 7.0);
    norm_pts = pts_normalize(f->corners);
    for (auto c : f->corners) {
        f->kps.push_back(cv::KeyPoint(c, 20.f));
    }

    for (int i = 0; i < norm_pts.rows; ++i) {
        f->kps.push_back(cv::KeyPoint(norm_pts.at<float>(i, 0),
                                      norm_pts.at<float>(i, 1), 20.f));
    }

    this->detector->compute(f->image, f->kps, f->descriptors);
}

} // namespace fe_extract
