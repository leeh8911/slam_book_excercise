#include "src/eigenMatrix.h"

#include <iostream>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

#define MAXRIX_SIZE (50U)

int Chapter3(void) {
    Eigen::Matrix<float, 2, 3> matrix_23;
    std::cout << "Create 2x3 Matrix" << std::endl;
    std::cout << matrix_23 << std::endl;

    Eigen::Vector3d v_3d;
    std::cout << "Create 3d vector" << std::endl;
    std::cout << v_3d << std::endl;

    return 0;
}
