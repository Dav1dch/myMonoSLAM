#ifndef TYPE_H
#define TYPE_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/flann.hpp"
#include "sophus/se3.hpp"

class Frame {
  public:
    cv::Mat image, descriptors, essential_matrix, R, t;
    cv::flann::Index kd;
    std::vector<cv::Point2d> corners;
    std::vector<cv::KeyPoint> kps;
    std::vector<cv::KeyPoint> kps_all;
    std::vector<cv::Point2d> points1, points2, pts1, pts2;
    std::vector<int> point_idxs, match_idxs;
    Sophus::SE3d T;
    int idx;
    Frame()
        : R(cv::Mat::eye(3, 3, CV_32F)), t(cv::Mat::zeros(3, 1, CV_32F)),
          idx(0) {
        Eigen::Matrix3d Rotation;
        Eigen::Vector3d Translation;
        cv::cv2eigen(R, Rotation);
        cv::cv2eigen(t, Translation);
        T = Sophus::SE3d(Rotation, Translation);
    }
};

class Point {
  public:
    std::vector<int> frames;
    std::vector<int> idxs;
    int idx;
    cv::Mat pose;
    Sophus::SE3d T;
    void add_observation(int frame_idx, int kps_idx, Frame *f);
    Point(){};
};

class PointMap {
  public:
    std::vector<Point *> points;
    std::vector<Frame *> frames;
    void add_frame(Frame *f);
    void add_point(Point *p);
    double orb_distance(Frame *f, int point_idx, int kps_idx);
    PointMap(){};
};

struct renderFrame {
  public:
    Frame *f;
    std::vector<Point> points;
};

#endif
