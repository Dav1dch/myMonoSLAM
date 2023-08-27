#ifndef OPTIMIZE_H
#define OPTIMIZE_H

const static double fx = 517.3, fy = 516.5, cx = 318.6, cy = 255.3;
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/edge_project_xyz2uv.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/se3quat.h>
#include <iostream>

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "type.h"
using namespace Eigen;
using namespace std;
using namespace Sophus;

void SolveBa(PointMap *map);
/// 姿态和内参的结构
struct Pose {
    Pose() {}

    /// set from given data address
    explicit Pose(SE3d *T) {
        rotation = SO3d(T->rotationMatrix());
        translation = T->translation();
    }

    /// 将估计值放入内存
    void set_to(Frame *f) {
        SE3d T;
        T.so3() = rotation;
        T.translation() = translation;
        f->T = T;
        f->t.at<double>(0) = translation.x();
        f->t.at<double>(1) = translation.y();
        f->t.at<double>(2) = translation.z();
        Eigen::MatrixXd rm = T.rotationMatrix();
        cv::eigen2cv(rm, f->R);
    }

    SO3d rotation;
    Vector3d translation = Vector3d::Zero();
};

/// 位姿加相机内参的顶点，9维，前三维为so3，接下去为t, f, k1, k2
class VertexPose : public g2o::BaseVertex<6, Pose> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPose() {}

    virtual void setToOriginImpl() override { _estimate = Pose(); }

    virtual void oplusImpl(const double *update) override {
        _estimate.rotation =
            SO3d::exp(Vector3d(update[0], update[1], update[2])) *
            _estimate.rotation;
        _estimate.translation += Vector3d(update[3], update[4], update[5]);
    }

    /// 根据估计值投影一个点
    Vector2d project(const Vector3d &point) {
        Vector3d pc = _estimate.rotation * point + _estimate.translation;
        pc = pc / pc[2];
        return Vector2d(fx * pc[0] + cx, fy * pc[1] + cy);
    }

    // virtual bool read(istream &in) {}
    //
    // virtual bool write(ostream &out) const {}
};

class VertexPoint : public g2o::BaseVertex<3, Vector3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}

    virtual void setToOriginImpl() override { _estimate = Vector3d(0, 0, 0); }

    virtual void oplusImpl(const double *update) override {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }

    // virtual bool read(istream &in) {}
    //
    // virtual bool write(ostream &out) const {}
};

class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Vector2d, VertexPose, VertexPoint> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override {
        auto v0 = (VertexPose *)_vertices[0];
        auto v1 = (VertexPoint *)_vertices[1];
        auto proj = v0->project(v1->estimate());
        _error = proj - _measurement;
    }

    // use numeric derivatives
    // virtual bool read(istream &in) {}
    //
    // virtual bool write(ostream &out) const {}
};
#endif
