#ifndef MATCH_H
#define MATCH_H

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "tools.h"
#include "type.h"
#include <iostream>
#include <set>

class feature_matcher {
  public:
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat K;
    std::vector<cv::DMatch> matches;
    feature_matcher();
    void match_frames(Frame *f1, Frame *f2);
    double width, height, tresholdDist;
};

#endif
