#pragma once

#include "API/Trajectory/trajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"

extern Eigen::MatrixXcd ComputeFFT2(const Eigen::MatrixXd& signal);

// Computes the SAL crieterion
// SAL: Spectral Arc Length
extern double sal_performance(const Move3D::Trajectory& trajectory,
                              const std::vector<Move3D::Joint*>& joints);

// Computes the difference in SAL between t0 and the test set
std::vector<double> sal_sequences_similarities(
    const Move3D::Trajectory& t0,
    const std::vector<Move3D::Trajectory>& t_tests,
    const std::vector<Move3D::Joint*>& joints);
