#include "optimize.h"

void SolveBa(PointMap *map) {
    const int point_size = map->points.size();
    const int frame_size = map->frames.size();

    // pose dimension 6, landmark is 3
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    // use LM

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        make_unique<BlockSolverType>(make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    /// build g2o problem
    /// VertexSE3Expmap
    vector<VertexPose *> vertex_pose;
    vector<VertexPoint *> vertex_points;
    vector<g2o::VertexSE3Expmap *> vertex_SE3;
    vector<g2o::VertexPointXYZ *> vertex_points_xyz;
    for (int i = 0; i < frame_size; ++i) {
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setId(i);
        // v->setEstimate(Pose(&map->frames[i]->T));
        v->setEstimate(g2o::SE3Quat(map->frames[i]->T.rotationMatrix(),
                                    map->frames[i]->T.translation()));
        optimizer.addVertex(v);
        // vertex_pose.push_back(v);
        vertex_SE3.push_back(v);
    }
    for (int i = 0; i < point_size; ++i) {
        // VertexPoint *v = new VertexPoint();
        g2o::VertexPointXYZ *v = new g2o::VertexPointXYZ();
        v->setId(i + frame_size);
        v->setEstimate(Vector3d(map->points[i]->pose.at<double>(0),
                                map->points[i]->pose.at<double>(1),
                                map->points[i]->pose.at<double>(2)));
        // g2o在BA中需要手动设置待Marg的顶点
        v->setMarginalized(true);
        optimizer.addVertex(v);
        // vertex_points.push_back(v);
        vertex_points_xyz.push_back(v);
    }

    std::cout << "after" << std::endl;
    // edge
    for (int i = 0; i < point_size; ++i) {
        for (int j = 0; j < map->points[i]->frames.size(); ++j) {

            // EdgeProjection *edge = new EdgeProjection;
            g2o::EdgeSE3ProjectXYZ *edge = new g2o::EdgeSE3ProjectXYZ();
            edge->setVertex(
                0, dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(i)));
            edge->setVertex(
                1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(j)));
            // edge->setVertex(1, vertex_SE3[j]);
            edge->setMeasurement(Vector2d(
                map->frames[j]->kps_all[map->points[i]->idxs[j]].pt.x,
                map->frames[j]->kps_all[map->points[i]->idxs[j]].pt.y));
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setParameterId(0, 0);
            optimizer.addEdge(edge);
        }
    }
    std::cout << "after" << std::endl;

    optimizer.initializeOptimization();
    optimizer.optimize(40);

    // set to bal problem
    for (int i = 0; i < frame_size; ++i) {
        auto vertex = vertex_SE3[i];
        auto estimate = vertex->estimate();
        SE3d T(estimate.rotation(), estimate.translation());
        map->frames[i]->T = T;
        // estimate.set_to(map->frames[i]);
        map->frames[i]->t.at<double>(0) = estimate.translation().x();
        map->frames[i]->t.at<double>(1) = estimate.translation().y();
        map->frames[i]->t.at<double>(2) = estimate.translation().z();
        Eigen::MatrixXd rm = T.rotationMatrix();
        cv::eigen2cv(rm, map->frames[i]->R);
    }
    for (int i = 0; i < point_size; ++i) {
        auto vertex = vertex_points_xyz[i];
        for (int k = 0; k < 3; ++k)
            map->points[i]->pose.at<double>(k) = vertex->estimate()[k];
    }
}
