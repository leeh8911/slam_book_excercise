// Eigen core
#include <Eigen/Core>
// Algebraic operations of dense matrices (inverse, eigenvalues, etc. )
#include <Eigen/Dense>
// Geometry
#include <Eigen/Geometry>

#include <cmath>
#include <ctime>
#include <gtest/gtest.h>
#include <iostream>
#include <pangolin/pangolin.h>
#include <string>
#include <unistd.h>

#include "src/DrawTrajectory.h"

#define MATRIX_SIZE (50)

class EigenLibTutorial : public testing::Test {
  public:
    void SetUp() override {}
    void TearDown() override {}
};

// Slambook page 38
TEST_F(EigenLibTutorial, MatrixBasic) {
    Eigen::Matrix<float, 2, 3> matrix_23;
    matrix_23.setZero();
    std::cout << "Create 2 x 3 float type matrix : \n"
              << matrix_23 << std::endl;

    Eigen::Vector3d v_3d;
    std::cout << "Create 3 dimensional double type vector : \n"
              << v_3d << std::endl;
    Eigen::Matrix<float, 3, 1> vd_3d;
    vd_3d.setZero();
    std::cout << "Create 3 dimensional float type vector(using Matrix) : \n"
              << vd_3d << std::endl;

    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    std::cout << "Create Matrix 3 x 3 double type matrix, then set to zero : \n"
              << matrix_33 << std::endl;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    matrix_dynamic.setZero();
    std::cout << "Create dynamic size matrix : \n " << matrix_dynamic
              << std::endl;
    Eigen::MatrixXd matrix_x;
    std::cout << "Create dynamic size matrix more simple : \n"
              << matrix_x << std::endl;

    matrix_23 << 1, 2, 3, 4, 5, 6;
    std::cout << "Set matrix_23 from 1 to 6 : \n" << matrix_23 << std::endl;

    std::cout << "Access matrix element using () operator \n";
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << matrix_23(i, j) << "\t";
        }
        std::cout << std::endl;
    }

    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;
    std::cout << "v_3d transposed : " << v_3d.transpose() << std::endl;
    std::cout << "vd_3d transposed : " << vd_3d.transpose() << std::endl;

    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "[1,2,3; 4,5,6] x [3,2,1] = " << result.transpose()
              << std::endl;

    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    std::cout << "[1,2,3; 4,5,6] x [4,5,6] = " << result2.transpose()
              << std::endl;
}

// Slambook page 38
TEST_F(EigenLibTutorial, LinearAlgebra) {
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    matrix_33 = Eigen::Matrix3d::Random();

    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;
    std::cout << "sum: \n" << matrix_33.sum() << std::endl;
    std::cout << "trace: \n" << matrix_33.trace() << std::endl;
    std::cout << "times 10: \n " << matrix_33 * 10 << std::endl;
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;
    std::cout << "determinant: \n" << matrix_33.determinant() << std::endl;

    // Eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;

    // Solving equations
    // solve : matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose(); // Guarantee semi-positive definite
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock(); //timing
    // Direct inverse
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "time of normal inverse is " << 1000. * static_cast<double>(clock() - time_stt) / static_cast<double>(CLOCKS_PER_SEC) << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    time_stt = clock();
    // QR decomposition
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    std::cout << "time of Qr decomposition is " << 1000. * static_cast<double>(clock() - time_stt) / static_cast<double>(CLOCKS_PER_SEC) << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    time_stt = clock();
    // cholesky decomposition
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "time of ldlt is " << 1000. * static_cast<double>(clock() - time_stt) / static_cast<double>(CLOCKS_PER_SEC) << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;
}


// Slambook page 50
TEST_F(EigenLibTutorial, Geometry) {
    // Eigen/geometry module provides a variety of rotation and translation representations
    // 3d rotation matrix directly using Matrix3d or Matrix 3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    // The rotation vector uses AngleAxis, the underlying layer is not directly Matrix, but the operation can be treated as a matrix (because the operator is overloaded)
    Eigen::AngleAxisd rotation_vector(M_PI / 4., Eigen::Vector3d(0, 0, 1)); // rotate 45 degrees along the Z-axis
    std::cout.precision(3);
    std::cout << "rotation vector = " << rotation_vector.angle() << std::endl;
    std::cout << "rotation vector matrix() = \n" << rotation_vector.matrix() << std::endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    std::cout << "rotation matrix = \n" << rotation_matrix << std::endl;
    
    // coordinate transformation with AngleAxis
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    std::cout << "(1, 0, 0) after rotation (by angle axis) = " << v_rotated.transpose() << std::endl;

    // Or use a rotation matrix
    v_rotated = rotation_matrix * v;
    std::cout << "(1, 0, 0) after rotation (by matrix) = " << v_rotated.transpose() << std::endl;

    // Euler angle: You can convert the rotation matrix directly into Euler angles
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX order, ie roll pitch yaw order
    std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl;

    // Euclidean transformation matrix using Eigen::Isometry
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); // Although called 3d, it is essentially a 4*4 matrix
    T.rotate(rotation_vector); // Rotate accoring to rotation_vector)

    T.pretranslate(Eigen::Vector3d(1, 3, 4)); // Set the translation vector to (1, 3, 4)
    std::cout << "Transform matrix = \n" << T.matrix() << std::endl;

    // Use the transformation matrix for coordinate transformation
    Eigen::Vector3d v_transformed = T * v; // Equivalent to R * v + t
    std::cout << "v_transformed = " << v_transformed.transpose() << std::endl;
    
    // For affine and projective transformations, use Eigen::Affine3d and Eigen::Projective3d

    // Quaternion
    // You can assign AngleAxis directly to quaternions, and vice versa
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl;

    // Rotation a vector with a quaternion and use overloaded multiplication
    v_rotated = q * v; // Note that the math is qvq^{-1}
    std::cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << std::endl;
    // expressed by regular vector multiplication, it should be calculated as follows
    std::cout << "should be equal to " << (q * Eigen::Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << std::endl;
}

TEST_F(EigenLibTutorial, CoordinateTransform) {
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -1, 0.2);
    q1.normalize();
    q2.normalize();
    Eigen::Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
    Eigen::Vector3d p1(0.5, 0, 0.2);

    Eigen::Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    Eigen::Vector3d p2 = T2w * T1w.inverse() * p1;
    std::cout << std::endl << p2.transpose() << std::endl;
}

