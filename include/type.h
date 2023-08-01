#ifndef TYPE_H
#define TYPE_H
#include "opencv2/core.hpp"

struct frame_ {
    cv::Mat image, descriptors, essential_matrix;
    std::vector<cv::Point2f> corners;
    std::vector<cv::KeyPoint> kps;
    std::vector<cv::Point2f> points1, points2;
};

#endif
