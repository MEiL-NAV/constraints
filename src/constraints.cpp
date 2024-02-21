#include "constraints.h"

Eigen::Vector<float, 13> constraints(const Eigen::Vector<float, 13> &state) {
    // TODO: implement
    return Eigen::Vector<float, 13>::Zero();
}

Eigen::Matrix<float, 13, 13> constraints_derivative(const Eigen::Vector<float, 13> &state) {
    // TODO: implement
    return Eigen::Matrix<float, 13, 13>::Identity();
}
