#ifndef FRAME_H
#define FRAME_H

#include "extract.h"
#include "match.h"
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>

#include <Eigen/src/Geometry/Quaternion.h>

namespace frame {
class frame {
  public:
    std::vector<frame_ *> frames;
    bool isInitialized;
    std::vector<std::vector<float>> posees;
    int idx;

    fe_extract::feature_extractor extractor;
    fe_matcher::feature_matcher matcher;

    void process_frame_orb(cv::Mat frame);
    void process_frame_direct(cv::Mat frame);
    void extractRt(frame_ *f1, frame_ *f2);
    frame();
};

} // namespace frame
  //
#endif
