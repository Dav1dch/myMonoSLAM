#ifndef PROCESS_H
#define PROCESS_H

#include "extract.h"
#include "global.h"
#include "match.h"
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/flann.hpp"
#include "optimize.h"
#include "render.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "tools.h"
#include "type.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <iostream>

class process {
  public:
    PointMap *map;

    bool isInitialized;
    std::vector<std::vector<double>> posees;
    int idx, width, height;

    feature_extractor extractor;
    feature_matcher matcher;
    cv::Mat K;

    void process_frame_orb(cv::Mat frame, viewer *v);
    void process_frame_direct(cv::Mat frame);
    void extractRt(Frame *f1, Frame *f2);
    process(PointMap *map);
};

//
#endif
