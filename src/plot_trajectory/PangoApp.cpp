// Eigen core
#include <Eigen/Core>
// Algebraic operations of dense matrices (inverse, eigenvalues, etc. )
#include <Eigen/Dense>
// Geometry
#include <Eigen/Geometry>

#include <iostream>
#include <pangolin/pangolin.h>
#include <string>
#include <unistd.h>

#include "DrawTrajectory.h"

int main() {
    std::string trajectory_file = "/develop/data/ch3_trajectory.txt";

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        poses;
    std::ifstream fin(trajectory_file);

    if (!fin) {
        std::cout << "cannot find trajectory file at " << trajectory_file
                  << std::endl;
        return 0;
    }

    while (!fin.eof()) {
        double time = NAN, tx = NAN, ty = NAN, tz = NAN, qx = NAN, qy = NAN,
               qz = NAN, qw = NAN;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    std::cout << "read total " << poses.size() << " pose entries" << std::endl;

    DrawTrajectory(poses);

    return 0;
}