#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"
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
    cv::Ptr<cv::DescriptorMatcher> matcher;
    bool isInitialized;
    feature_extractor();
    void extract(cv::Mat frame, std::vector<cv::Point2f> &corners,
                 cv::Mat descriptors);
};

cv::Mat pt_normalize(std::vector<cv::Point2f> pts);
cv::Mat pt_denormalize(std::vector<cv::Point2f> pts);

} // namespace fe_extract
