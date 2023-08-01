#ifndef FRAME_H
#define FRAME_H

#include "extract.h"
#include "match.h"
#include "opencv2/core.hpp"
#include <iostream>

namespace frame {
class frame {
  public:
    std::vector<frame_ *> frames;
    bool isInitialized;
    fe_extract::feature_extractor extractor;
    fe_matcher::feature_matcher matcher;

    void process_frame(cv::Mat frame);
    frame();
};

} // namespace frame
  //
#endif
