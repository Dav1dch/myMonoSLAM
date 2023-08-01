#include "tools.h"

cv::Mat pts_normalize(std::vector<cv::Point2f> pts) {
    cv::Mat mat(2, pts.size(), CV_32F, pts.data());
    cv::Mat col = cv::Mat::ones(1, mat.cols, CV_32F);

    cv::vconcat(mat, col, mat);

    cv::Mat K(cv::Size(3, 3), CV_32F);
    K = (cv::Mat_<float>(3, 3) << 726.21081542969, 0.0, 359.2048034668, 0.0,
         726.21081542969, 202.47247314453, 0.0, 0.0, 1.0);
    cv::Mat K_inv = K.inv();
    cv::Mat res = K_inv * mat;
    res = res.t();

    return res.t();
}
