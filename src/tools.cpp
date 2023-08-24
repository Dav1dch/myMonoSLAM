#include "tools.h"
#include <iostream>

Eigen::Matrix4f Homogeneous_matrix(Eigen::MatrixXf mat) {
    Eigen::Matrix4f homo_matrix = Eigen::Matrix4f::Zero();
    homo_matrix.block<3, 4>(0, 0) = mat;
    homo_matrix(3, 3) = 1;
    return homo_matrix;
}
