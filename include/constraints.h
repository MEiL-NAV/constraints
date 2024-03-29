#pragma once
#include <eigen3/Eigen/Dense>

extern Eigen::VectorXf constraints(const Eigen::Vector<float, 15> &state);
extern Eigen::MatrixXf constraints_derivative(const Eigen::Vector<float, 15> &state);
