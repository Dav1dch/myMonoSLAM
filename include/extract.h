#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace fe_extract {
class feature_extractor {
  public:
    Mat last, descriptors_;
    vector<Point2f> corners_;
    vector<KeyPoint> kps_;
    vector<DMatch> matches;
    Ptr<DescriptorMatcher> matcher;
    bool isInitialized;
    feature_extractor();
    void extract(Mat frame, vector<Point2f> &corners, Mat descriptors);
};

} // namespace fe_extract
