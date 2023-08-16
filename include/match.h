#ifndef MATCH_H
#define MATCH_H

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "tools.h"
#include "type.h"
#include <iostream>
#include <set>

namespace fe_matcher {
class feature_matcher {
  public:
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat K;
    std::vector<cv::DMatch> matches;
    feature_matcher();
    void match_frames(frame_ *f1, frame_ *f2);
    float width, height, tresholdDist;
};
} // namespace fe_matcher

#endif

cv::Mat pts_normalize(std::vector<cv::Point2f> pts);
