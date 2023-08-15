#include "tools.h"
#include <iostream>

cv::Mat pts_normalize(std::vector<cv::Point2f> pts) {
    Eigen::Vector3f vec;
    Eigen::Matrix<float, 3, 3> K, K_inv;
    K << 726.21081542969, 726.21081542969, 359.2048034668, 202.47247314453;
    K_inv = K.inverse();
    std::vector<float> res_vec;

    for (auto p : pts) {
        Eigen::Vector3f ho_vec;
        ho_vec << p.x, p.y, 1.0;
        ho_vec = K_inv * ho_vec;
        // std::vector<float> ho_p({ho_vec(0), ho_vec(1), ho_vec(2)});
        res_vec.push_back(ho_vec(0));
        res_vec.push_back(ho_vec(1));
    }

    cv::Mat res(pts.size(), 2, CV_32F, res_vec.data());
    return res;
}
