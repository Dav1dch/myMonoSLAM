#ifndef EXTRACT_H
#define EXTRACT_H

#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "tools.h"
#include "type.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

namespace fe_extract {
class feature_extractor {
  public:
    cv::Mat last, descriptors_;
    std::vector<cv::Point2f> corners_;
    std::vector<cv::KeyPoint> kps_;
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::ORB> detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    bool isInitialized;
    feature_extractor();
    void extract(cv::Mat frame, frame_ *f);
};

} // namespace fe_extract

#endif
