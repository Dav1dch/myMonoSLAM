#include "type.h"

void Point::add_observation(int frame_idx, int kps_idx, Frame *f) {
    f->point_idxs[kps_idx] = this->idx;
    this->frames.push_back(frame_idx);
    this->idxs.push_back(kps_idx);
}

void PointMap::add_point(Point *p) {
    p->idx = this->points.size();
    this->points.push_back(p);
}

void PointMap::add_frame(Frame *f) { this->frames.push_back(f); }
