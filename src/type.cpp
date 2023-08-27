#include "type.h"

void Point::add_observation(int frame_idx, int kps_idx, Frame *f) {
    f->point_idxs[kps_idx] = this->idx;
    this->frames.push_back(frame_idx);
    this->idxs.push_back(kps_idx);
}

double PointMap::orb_distance(Frame *f, int point_idxs, int kps_idx) {
    // norm(descriptors1,descriptors2,NORM_HAMMING)
    double min_distance = 10000.0f;
    for (int i = 0; i < this->points[point_idxs]->frames.size(); i++) {
        double tmp_distance = cv::norm(
            f->descriptors.row(kps_idx),
            this->frames[this->points[point_idxs]->frames[i]]->descriptors.row(
                this->points[point_idxs]->idxs[i]),
            cv::NORM_HAMMING);
        min_distance =
            tmp_distance < min_distance ? tmp_distance : min_distance;
    }
    return min_distance;
}

void PointMap::add_point(Point *p) {
    p->idx = this->points.size();
    this->points.push_back(p);
}

void PointMap::add_frame(Frame *f) { this->frames.push_back(f); }
