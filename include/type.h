#ifndef TYPE_H
#define TYPE_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "sophus/se3.hpp"

struct frame_ {
    cv::Mat image, descriptors, essential_matrix, R, t;
    std::vector<cv::Point2f> corners;
    std::vector<cv::KeyPoint> kps;
    std::vector<cv::KeyPoint> kps_all;
    std::vector<cv::Point2f> points1, points2;
    Sophus::SE3f T;
    int idx;
    frame_()
        : R(cv::Mat::eye(3, 3, CV_64F)), t(cv::Mat::zeros(3, 1, CV_64F)),
          idx(0) {
        Eigen::Matrix3f Rotation;
        Eigen::Vector3f Translation;
        cv::cv2eigen(R, Rotation);
        cv::cv2eigen(t, Translation);
        T = Sophus::SE3f(Rotation, Translation);
    }
};

struct Point {
    std::vector<int> frames;
    cv::Mat pose;
    std::vector<int> idxs;
};

struct renderFrame {
    frame_ *f;
    std::vector<Point> points;
};

#endif
