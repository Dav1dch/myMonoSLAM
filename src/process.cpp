#include "process.h"
#include <iostream>

process::process(PointMap *map) {
    this->isInitialized = false;
    this->idx = 0;
    this->width = 640;
    this->height = 480;
    this->map = map;
}

void process::process_frame_orb(cv::Mat image_f, viewer *v) {

    Frame *cur_frame = new Frame();
    cur_frame->idx = this->map->frames.size();

    this->extractor.extract(image_f, cur_frame);

    this->map->frames.push_back(cur_frame);
    int cur = this->map->frames.size();
    if (cur == 1) {
        this->isInitialized = true;

        cv::Mat draw;
        this->map->frames[0]->image.copyTo(draw);

        for (auto p : this->map->frames.at(0)->kps) {
            circle(draw, cv::Point2f(p.pt), 3, {0, 255, 0});
        } // for loop

        cv::imshow("Frame", draw);
        // cv::waitKey(0);
        this->isInitialized = true;
    } else {
        this->matcher.match_frames(this->map->frames[cur - 2],
                                   this->map->frames[cur - 1]);
        cv::Mat draw, draw_l, twin;
        Frame *cur_f, *last_f;

        cur_f = this->map->frames[cur - 1];
        last_f = this->map->frames[cur - 2];
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
        cv::Mat TMatr1, TMatr2, projMatr2, projMatr1;

        // construct Transform matrix
        cv::hconcat(last_f->R, last_f->t, TMatr1);
        cv::hconcat(cur_f->R, cur_f->t, TMatr2);

        // pass data to render by pangolin
        renderFrame rf;

        rf.f = cur_f;

        // construct homo pose matrix
        Eigen::MatrixXf pose1, pose2;
        Eigen::Matrix4f pose1_homogenous, pose2_homogenous;
        cv::cv2eigen(TMatr1, pose1);
        cv::cv2eigen(TMatr2, pose2);

        pose1_homogenous = Homogeneous_matrix(pose1);
        pose2_homogenous = Homogeneous_matrix(pose2);

        Eigen::MatrixXf K, T1, T2;
        cv::cv2eigen(matcher.K, K);
        T1 = K * pose1;
        T2 = K * pose2;
        cv::eigen2cv(Eigen::MatrixXf(T1.block<3, 4>(0, 0)), projMatr1);
        cv::eigen2cv(Eigen::MatrixXf(T2.block<3, 4>(0, 0)), projMatr2);

        cv::triangulatePoints(projMatr2, projMatr1, cur_f->pts1, last_f->pts2,
                              pts4d);
        pts4dt = pts4d.t();

        // add points observation to map
        if (this->map->points.size() == 0) {
            for (auto i = 0; i < pts4dt.rows; i++) {
                pts4dt.row(i) = pts4dt.row(i) / pts4dt.at<float>(i, 3);
                // std::cout << pts4dt.row(i) << std::endl;
                Eigen::Vector4f p;
                cv::cv2eigen(pts4dt.row(i).t(), p);
                Eigen::VectorXf pl1, pl2, points;
                pl1 = pose1_homogenous * p;
                pl2 = pose2_homogenous * p;

                if (pl1(2) <= 0 || pl2(2) <= 0) {
                    continue;
                }
                // std::cout << pose.t() << std::endl;
                Point *pt = new Point();
                this->map->add_point(pt);
                pt->idx = this->map->points.size();
                pt->add_observation(this->map->frames.size() - 2,
                                    this->matcher.matches[i].trainIdx, cur_f);
                pt->add_observation(this->map->frames.size() - 1,
                                    this->matcher.matches[i].queryIdx, cur_f);
                pt->pose = pts4dt.row(i).t();
                rf.points.push_back(*pt);
            }
        } else {
            // to track points which are observed in multi frames
            std::vector<int> observed_point_idxs;
            std::vector<cv::Point2f> observed_points;
            for (int i = 0; i < this->map->points.size(); i++) {
                Eigen::MatrixXf point_pose, projectPoint;
                cv::cv2eigen(this->map->points[i]->pose, point_pose);
                projectPoint =
                    K * (pose1_homogenous * point_pose).block<3, 1>(0, 0);
                projectPoint = projectPoint / projectPoint(2);

                // collect the observed points
                if (projectPoint(0) > 0 && projectPoint(0) < this->width &&
                    projectPoint(1) > 0 && projectPoint(1) < this->height) {
                    observed_point_idxs.push_back(i);
                    observed_points.push_back(
                        cv::Point2f(projectPoint(0), projectPoint(1)));
                }
            }
            for (int i = 0; i < observed_points.size(); i++) {

                // 预设knnSearch所需参数及容器
                int queryNum = 3; // 用于设置返回邻近点的个数
                std::vector<float> vecQuery(2);      // 存放查询点的容器
                std::vector<int> vecIndex(queryNum); // 存放返回的点索引
                std::vector<float> vecDist(queryNum); // 存放距离
                cv::flann::SearchParams params(32); // 设置knnSearch搜索参数

                // KD树knn查询
                vecQuery = {observed_points[i].x, observed_points[i].y};
                cur_f->kd->radiusSearch(vecQuery, vecIndex, vecDist, 2.0, 3);
                for (auto item : vecDist) {
                    if (item != 0) {
                        std::cout << item << std::endl;
                    }
                }
            }
        }
        std::cout << rf.points.size() << std::endl;
        v->renderQueue.push_back(rf);

        cv::drawMatches(draw, cur_f->kps_all, draw_l, last_f->kps_all,
                        this->matcher.matches, twin);
        v->co->resume();
        cv::imshow("Frame", twin);
        // cv::waitKey(0);

        // cv::Mat pose;
        // cv::hconcat(r, t, pose);
    }
}

void process::extractRt(Frame *cur_f, Frame *last_f) {
    cv::Mat r, t;
    Eigen::Matrix3f Rotation;
    Eigen::Vector3f Translation;

    cv::recoverPose(cur_f->essential_matrix, cur_f->pts1, last_f->pts2,
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
    cv::eigen2cv(T.translation(), cur_f->t);
    cv::eigen2cv(T.rotationMatrix(), cur_f->R);

    std::cout << T.matrix()
              << std::endl
              // << "T rotation" << std::endl
              // << q << std::endl
              // << "T Translation: " << std::endl
              << T.translation().transpose() << std::endl;
    this->posees.push_back({float(this->idx++), tr.x(), tr.y(), tr.z(), q.x(),
                            q.y(), q.z(), q.w()});
}
