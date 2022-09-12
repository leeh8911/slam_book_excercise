#ifndef _DEVELOP_SRC_PLOT_TRAJECTORY_DRAWTRAJECTORY_H
#define _DEVELOP_SRC_PLOT_TRAJECTORY_DRAWTRAJECTORY_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <vector>

void DrawTrajectory(std::vector<Eigen::Isometry3d,
                                Eigen::aligned_allocator<Eigen::Isometry3d>>);

#endif // _DEVELOP_SRC_PLOT_TRAJECTORY_DRAWTRAJECTORY_H