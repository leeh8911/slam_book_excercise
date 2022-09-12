#include "sophus/se3.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>

TEST(SophusTest, SophusExcercise) {
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Eigen::Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);

    std::cout << "SO(3) from matrix: \n" << SO3_R.matrix() << std::endl;
    std::cout << "SO(3) from quaternion: \n" << SO3_q.matrix() << std::endl;
    std::cout << "they are equal!" << std::endl;
    EXPECT_EQ(SO3_R.matrix(), SO3_q.matrix());

    Eigen::Vector3d so3 = SO3_R.log();
    std::cout <<"so3 = " << so3.transpose() << std::endl;
    std::cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << std::endl;
    std::cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl;

    Eigen::Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    std::cout << "SO3 updated = \n" << SO3_updated.matrix() << std::endl;

    std::cout << "********************************************" << std::endl;
    Eigen::Vector3d t(1, 0, 0);
    Sophus::SE3d SE3_Rt(R, t);
    Sophus::SE3d SE3_qt(q, t);
    std::cout << "SE3 from R, t= \n" << SE3_Rt.matrix() << std::endl;
    std::cout << "SE3 from q, t= \n" << SE3_qt.matrix() << std::endl;

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    std::cout << "se3 = " << se3.transpose() << std::endl;
    std::cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) <<std::endl;
    std::cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl;

    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4f;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    std::cout << "SE3 updated = \n" << SE3_updated.matrix() << std::endl;
    
}