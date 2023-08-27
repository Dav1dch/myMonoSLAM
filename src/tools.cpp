#include "tools.h"
#include <iostream>

Eigen::Matrix4d Homogeneous_matrix(Eigen::MatrixXd mat) {
    Eigen::Matrix4d homo_matrix = Eigen::Matrix4d::Zero();
    homo_matrix.block<3, 4>(0, 0) = mat;
    homo_matrix(3, 3) = 1;
    return homo_matrix;
}
