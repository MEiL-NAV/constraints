#include "constraints.h"

Eigen::VectorXf constraints(const Eigen::Vector<float, 13> &state) {
    Eigen::VectorXf FI = Eigen::VectorXf(1);
    auto quaterion = state.segment<4>(6);
    FI(0) = quaterion.squaredNorm() - 1.0f;
    return FI;
}

Eigen::MatrixXf constraints_derivative(const Eigen::Vector<float, 13> &state) {
    Eigen::MatrixXf FId = Eigen::MatrixXf::Zero(1,13);
    auto quaterion = state.segment<4>(6);
    FId.block<1,4>(0,6) = 2.0f * quaterion.transpose();
    return FId;
}
