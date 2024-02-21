#pragma once
#include <eigen3/Eigen/Dense>

Eigen::Vector<float, 13> constraints(const Eigen::Vector<float, 13> &state);
Eigen::Matrix<float, 13, 13> constraints_derivative(const Eigen::Vector<float, 13> &state);
