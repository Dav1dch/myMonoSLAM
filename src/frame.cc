#include "frame.h"

namespace frame {
frame::frame() { this->isInitialized = false; }

void frame::process_frame(cv::Mat image_f) {
    // cv::Mat frame_;
    // cv::resize(frame, frame_, Size(540, 480));

    frame_ *cur_frame = new frame_();

    this->extractor.extract(image_f, cur_frame);
    this->frames.push_back(cur_frame);
    // cout << corners.size() << endl;
    int cur = this->frames.size();
    if (cur == 1) {
        this->isInitialized = true;

        cv::Mat draw;
        this->frames[0]->image.copyTo(draw);

        for (auto p : this->frames.at(0)->corners) {
            circle(draw, p, 3, {0, 255, 0});
        } // for loop

        cv::imshow("Frame", draw);
        this->isInitialized = true;
    } else {
        this->matcher.match_frames(this->frames[cur - 2],
                                   this->frames[cur - 1]);
        cv::Mat draw;
        frame_ *cur_f, *last_f;

        cur_f = this->frames[cur - 1];
        last_f = this->frames[cur - 2];
        cur_f->image.copyTo(draw);

        for (auto p : cur_f->corners) {
            circle(draw, p, 3, {0, 255, 0});
        } // for loop

        for (auto m : this->matcher.matches) {
            line(draw, cur_f->kps[m.queryIdx].pt, last_f->kps[m.trainIdx].pt,
                 {255, 0, 0}, 4);
        }

        cv::imshow("Frame", draw);
        cv::waitKey(0);
        cv::Mat r, t;
        cv::recoverPose(cur_f->essential_matrix, last_f->points2,
                        cur_f->points1, matcher.K, r, t);
        cv::Mat pose;
        cv::hconcat(r, t, pose);
        std::cout << "r :" << r << std::endl;
        std::cout << "t :" << t << std::endl;
        std::cout << "pose :" << pose << std::endl;
    }
}
} // namespace frame
