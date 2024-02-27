#include "constraints.h"

Eigen::VectorXf constraints(const Eigen::Vector<float, 13> &state) {
    Eigen::VectorXf FI = Eigen::VectorXf(2);
    auto q = state.segment<4>(6);
    FI(0) = q.squaredNorm() - 1.0f;
    FI(1) = 2 * (q(0) * q(3) + q(1) * q(2));
    return FI;
}

Eigen::MatrixXf constraints_derivative(const Eigen::Vector<float, 13> &state) {
    Eigen::MatrixXf FId = Eigen::MatrixXf::Zero(2,13);
    auto q = state.segment<4>(6);
    FId.block<1,4>(0,6) = 2.0f * q.transpose();
    FId(1,6) = 2 * q(3);
    FId(1,7) = 2 * q(2);
    FId(1,8) = 2 * q(1);
    FId(1,9) = 2 * q(0);
    return FId;
}
