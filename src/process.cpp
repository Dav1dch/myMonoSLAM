#include "process.h"

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
            circle(draw, cv::Point2d(p.pt), 3, {0, 255, 0});
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
        Eigen::MatrixXd pose1, pose2;
        Eigen::Matrix4d pose1_homogenous, pose2_homogenous;
        cv::cv2eigen(TMatr1, pose1);
        cv::cv2eigen(TMatr2, pose2);

        pose1_homogenous = Homogeneous_matrix(pose1);
        pose2_homogenous = Homogeneous_matrix(pose2);

        Eigen::MatrixXd K, T1, T2;
        cv::cv2eigen(matcher.K, K);
        T1 = K * pose1;
        T2 = K * pose2;
        cv::eigen2cv(Eigen::MatrixXd(T1.block<3, 4>(0, 0)), projMatr1);
        cv::eigen2cv(Eigen::MatrixXd(T2.block<3, 4>(0, 0)), projMatr2);

        cv::triangulatePoints(projMatr2, projMatr1, cur_f->pts1, last_f->pts2,
                              pts4d);
        pts4dt = pts4d.t();

        std::vector<bool> good_pts4d(pts4dt.rows, true);

        // add points observation to map
        if (this->map->points.size() != 0) {

            // to track points which are observed in multi frames
            std::vector<int> observed_point_idxs;
            std::vector<cv::Point2d> observed_points;
            for (int i = 0; i < this->map->points.size(); i++) {
                Eigen::MatrixXd point_pose, projectPoint;
                cv::cv2eigen(this->map->points[i]->pose, point_pose);
                projectPoint =
                    K * (pose1_homogenous * point_pose).block<3, 1>(0, 0);
                projectPoint = projectPoint / projectPoint(2);

                // collect the observed points
                if (projectPoint(0) > 0 && projectPoint(0) < this->width &&
                    projectPoint(1) > 0 && projectPoint(1) < this->height) {
                    observed_point_idxs.push_back(i);
                    observed_points.push_back(cv::Point2d(
                        round(projectPoint(0)), round(projectPoint(1))));
                }
            }

            // build kd tree
            std::vector<cv::Point2d> kps;
            for (int i = 0; i < cur_f->kps_all.size(); i++) {
                kps.push_back({cur_f->kps_all[i].pt.x, cur_f->kps_all[i].pt.y});
            }
            cv::Mat_<double> features(0, 2);

            for (auto &&point : kps) {

                // Fill matrix
                cv::Mat row =
                    (cv::Mat_<float>(1, 2) << static_cast<float>(point.x),
                     static_cast<float>(point.y));
                features.push_back(row);
            }
            cv::flann::KDTreeIndexParams indexParams(2);
            cv::flann::Index tree(features, indexParams);

            cv::Mat indices, dists;
            for (int i = 0; i < observed_points.size(); i++) {

                cv::Mat query = (cv::Mat_<float>(1, 2) << observed_points[i].x,
                                 observed_points[i].y);
                tree.knnSearch(query, indices, dists, 3,
                               cv::flann::SearchParams());
                for (int j = 0; j < 3; j++) {
                    if (cur_f->point_idxs[indices.at<int>(j)] != -1) {
                        double dist = this->map->orb_distance(
                            cur_f, observed_point_idxs[i], indices.at<int>(j));
                        if (dist <= 64) {
                            this->map->points[observed_point_idxs[i]]
                                ->add_observation(cur_f->idx,
                                                  indices.at<int>(j), cur_f);
                            good_pts4d[cur_f->match_idxs[indices.at<int>(j)]] =
                                false;
                        }
                    }
                }
            }
        }

        for (auto i = 0; i < pts4dt.rows; i++) {
            if (!good_pts4d[i]) {
                continue;
            }
            pts4dt.row(i) = pts4dt.row(i) / pts4dt.at<double>(i, 3);
            // std::cout << pts4dt.row(i) << std::endl;
            Eigen::Vector4d p;
            cv::cv2eigen(pts4dt.row(i).t(), p);
            Eigen::VectorXd pl1, pl2, points;
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
                                this->matcher.matches[i].trainIdx, last_f);
            pt->add_observation(this->map->frames.size() - 1,
                                this->matcher.matches[i].queryIdx, cur_f);
            pt->pose = pts4dt.row(i).t();
            rf.points.push_back(*pt);
        }
        if (cur_f->idx > 2) {

            SolveBa(this->map);
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
    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Translation;

    cv::recoverPose(cur_f->essential_matrix, cur_f->pts1, last_f->pts2,
                    matcher.K, r, t);
    cv::cv2eigen(r, Rotation);
    cv::cv2eigen(t, Translation);
    cur_f->R = r;
    cur_f->t = t;

    Sophus::SE3d T(Rotation, Translation);

    Sophus::SE3d T_last = last_f->T;
    T = T * T_last;
    cur_f->T = T;
    Eigen::Quaterniond q(T.rotationMatrix());
    Eigen::Vector3d tr = T.translation();
    cv::eigen2cv(T.translation(), cur_f->t);
    cv::eigen2cv(T.rotationMatrix(), cur_f->R);

    std::cout << T.matrix()
              << std::endl
              // << "T rotation" << std::endl
              // << q << std::endl
              // << "T Translation: " << std::endl
              << T.translation().transpose() << std::endl;
    this->posees.push_back({double(this->idx++), tr.x(), tr.y(), tr.z(), q.x(),
                            q.y(), q.z(), q.w()});
}
