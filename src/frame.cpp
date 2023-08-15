#include "frame.h"
#include <Eigen/src/Core/ArithmeticSequence.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>

namespace frame {
frame::frame() {
    this->isInitialized = false;
    this->idx = 0;
}

void frame::process_frame_orb(cv::Mat image_f) {
    // cv::Mat frame_;
    // cv::resize(frame, frame_, Size(540, 480));

    frame_ *cur_frame = new frame_();
    cur_frame->idx = this->frames.size();

    this->extractor.extract(image_f, cur_frame);
    this->frames.push_back(cur_frame);
    std::cout << cur_frame->corners.size() << std::endl;
    int cur = this->frames.size();
    if (cur == 1) {
        this->isInitialized = true;

        cv::Mat draw;
        this->frames[0]->image.copyTo(draw);

        for (auto p : this->frames.at(0)->kps) {
            circle(draw, cv::Point2f(p.pt), 3, {0, 255, 0});
        } // for loop

        cv::imshow("Frame", draw);
        cv::waitKey(0);
        this->isInitialized = true;
    } else {
        this->matcher.match_frames(this->frames[cur - 2],
                                   this->frames[cur - 1]);
        cv::Mat draw, draw_l, twin;
        frame_ *cur_f, *last_f;

        cur_f = this->frames[cur - 1];
        last_f = this->frames[cur - 2];
        cur_f->image.copyTo(draw);
        last_f->image.copyTo(draw_l);

        for (auto p : cur_f->kps) {
            circle(draw, p.pt, 3, {0, 255, 0});
        } // for loop

        for (auto m : this->matcher.matches) {
            line(draw, cur_f->kps_all[m.queryIdx].pt,
                 last_f->kps_all[m.trainIdx].pt, {255, 0, 0}, 4);
        }

        this->extractRt(cur_f, last_f);

        cv::Mat pts4d, pts4dt;
        cv::Mat projMatr1, projMatr2;

        // cv::hconcat(last_f->T.rotationMatrix(), last_f->T.translation(),
        //             projMatr1);
        cv::hconcat(last_f->R, last_f->t, projMatr1);
        cv::hconcat(cur_f->R, cur_f->t, projMatr2);

        cv::triangulatePoints(projMatr1, projMatr2, last_f->points2,
                              cur_f->points1, pts4d);

        pts4dt = pts4d.t();
        for (auto i = 0; i < pts4dt.rows; ++i) {
            pts4dt.row(i) = pts4dt.row(i) / pts4dt.at<float>(i, 3);
            Point pt;
            pt.frames.push_back(this->frames.size() - 2);
            pt.frames.push_back(this->frames.size() - 1);
            pt.idxs.push_back(this->matcher.matches[i].trainIdx);
            pt.idxs.push_back(this->matcher.matches[i].queryIdx);
            pt.pose = pts4dt.row(i);
            this->points.push_back(pt);
        }

        cv::drawMatches(draw, cur_f->kps_all, draw_l, last_f->kps_all,
                        this->matcher.matches, twin);
        cv::imshow("Frame", twin);
        // cv::waitKey(0);

        // cv::Mat pose;
        // cv::hconcat(r, t, pose);
    }
}

void frame::extractRt(frame_ *cur_f, frame_ *last_f) {
    cv::Mat r, t;
    Eigen::Matrix3f Rotation;
    Eigen::Vector3f Translation;

    cv::recoverPose(cur_f->essential_matrix, last_f->points2, cur_f->points1,
                    matcher.K, r, t);
    cv::cv2eigen(r, Rotation);
    cv::cv2eigen(t, Translation);
    cur_f->R = r;
    cur_f->t = t;

    Sophus::SE3f T(Rotation, Translation);

    Sophus::SE3f T_last = last_f->T;
    T = T * T_last;
    cur_f->T = T;
    Eigen::Quaternionf q(T.rotationMatrix());
    Eigen::Vector3f tr = T.translation();
    // cv::eigen2cv(T.matrix3x4()(Eigen::all, Eigen::seq(0, 2)), cur_f->R);
    // cv::eigen2cv(T.matrix3x4()(Eigen::all, Eigen::seq(3, 3)), cur_f->t);

    std::cout << T.matrix() << std::endl
              << "T rotation" << std::endl
              << q << std::endl
              << "T Translation: " << std::endl
              << T.translation() << std::endl;
    this->posees.push_back({float(this->idx++), tr.x(), tr.y(), tr.z(), q.x(),
                            q.y(), q.z(), q.w()});

    // cv::Mat last_pose, cur_pose;
    // cv::Mat zero_one = cv::Mat::zeros(1, 4, CV_64F);
    // zero_one.at<double>(3) = 1.0;

    // cv::hconcat(last_f->R, last_f->t, last_pose);
    // cv::vconcat(last_pose, zero_one, last_pose);

    // cv::hconcat(r, t, cur_pose);
    // cv::vconcat(cur_pose, zero_one, cur_pose);

    // cur_pose = cur_pose * last_pose;
    // cur_f->R = cur_pose.colRange(0, 3).rowRange(0, 3);
    // cur_f->t = cur_pose.col(3).rowRange(0, 3);
    // std::cout << cur_f->R << std::endl;
    // std::cout << cur_f->t << std::endl;
}
} // namespace frame
