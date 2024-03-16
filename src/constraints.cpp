#include "constraints.h"

inline float sd(float val)
{
    return val * val;
}

float distance_constraint(const Eigen::Vector<float, 13> &state,
                          const Eigen::Vector3f& stationary_joint,
                          const Eigen::Vector3f& moving_joint,
                          float distance) 
{
    float& x = state(0);
    float& y = state(1);
    float& z = state(2);
    float& q0 = state(6);
    float& qx = state(7);
    float& qy = state(8);
    float& qz = state(9);

    float& rax = moving_joint(0);
    float& ray = moving_joint(1);
    float& raz = moving_joint(2);

    float& rbx = stationary_joint(0);
    float& rby = stationary_joint(1);
    float& rbz = stationary_joint(2);

    return sd(rbx - x - rax*(2.0f * sd(q0) + 2.0f * sd(qx) - 1.0f) 
        + ray*(2.0f*q0*qz - 2.0f*qx*qy) 
        + raz*(2.0f*q0*qy - 2.0f*qx*qz)) 
        + sd(y - rby + ray*(2.0f*sd(q0) + 2.0f*sd(qy) - 1.0f) 
        + rax*(2*q0*qz + 2.0f*qx*qy) + raz*(2*q0*qx + 2.0f*qy*qz))
        + sd(z - rbz + raz*(2.0f*sd(q0) + 2.0f*sd(qz) - 1.0f) 
        + rax*(2.0f*q0*qy + 2.0f*qx*qz) - ray*(2.0f*q0*qx - 2.0f*qy*qz)) 
        - distance * distance;
}

Eigen::VectorXf distance_constraint_derivative(const Eigen::Vector<float, 13> &state,
                                              const Eigen::Vector3f& stationary_joint,
                                              const Eigen::Vector3f& moving_joint,
                                              float distance) 
{
    Eigen::VectorXf derivative = Eigen::VectorXf::Zero(13);
    float& x = state(0);
    float& y = state(1);
    float& z = state(2);
    float& q0 = state(6);
    float& qx = state(7);
    float& qy = state(8);
    float& qz = state(9);

    float& rax = moving_joint(0);
    float& ray = moving_joint(1);
    float& raz = moving_joint(2);

    float& rbx = stationary_joint(0);
    float& rby = stationary_joint(1);
    float& rbz = stationary_joint(2);

    derivative(0) = 2.0f * (x - rbx + rax*(2.0f * sd(q0) + 2.0f * sd(qx) - 1.0f))
        + 4.0f * ray * (q0*qz - qx*qy) 
        + 4.0f * raz * (q0*qy - qx*qz);

    derivative(1) = 2.0f * (y - rby + ray*(2.0f * sd(q0) + 2.0f * sd(qy) - 1.0f))
        + 4.0f * rax * (q0*qz + qx*qy) 
        + 4.0f * raz*(q0*qx + qy*qz);

    derivative(2) = 2.0f * (z - rbz + raz*(2.0f * sd(q0) + 2.0f * sd(qz) - 1.0f))
        + 4.0f * rax*(q0*qy + qx*qz) 
        - 4.0f * ray*(q0*qx - qy*qz);

    derivative(6) = 2.0f*(2.0f*qy*raz - 4*q0*rax + 2.0f*qz*ray)*(rbx - x - rax*(2.0f*sd(q0) + 2.0f*sd(qx) - 1.0f) 
        + ray*(2.0f*q0*qz - 2.0f*qx*qy) + raz*(2.0f*q0*qy - 2.0f*qx*qz)) 
        + 2.0f*(4*q0*ray + 2.0f*qx*raz + 2.0f*qz*rax)*(y - rby + ray*(2.0f*sd(q0) + 2.0f*sd(qy) - 1.0f) 
        + rax*(2.0f*q0*qz + 2.0f*qx*qy) + raz*(2.0f*q0*qx + 2.0f*qy*qz)) 
        + 2.0f*(4*q0*raz - 2.0f*qx*ray + 2.0f*qy*rax)*(z - rbz + raz*(2.0f*sd(q0) + 2.0f*sd(qz) - 1.0f) 
        + rax*(2.0f*q0*qy + 2.0f*qx*qz) - ray*(2.0f*q0*qx - 2.0f*qy*qz));

    derivative(7) = 2.0f*(2.0f*q0*raz + 2.0f*qy*rax)*(y - rby + ray*(2.0f*sd(q0) + 2.0f*sd(qy) - 1.0f) 
        + rax*(2.0f*q0*qz + 2.0f*qx*qy) + raz*(2.0f*q0*qx + 2.0f*qy*qz)) 
        - 2.0f*(4*qx*rax + 2.0f*qy*ray + 2.0f*qz*raz)*(rbx - x - rax*(2.0f*sd(q0) + 2.0f*sd(qx) - 1.0f) 
        + ray*(2.0f*q0*qz - 2.0f*qx*qy) + raz*(2.0f*q0*qy - 2.0f*qx*qz)) 
        - 2.0f*(2.0f*q0*ray - 2.0f*qz*rax)*(z - rbz + raz*(2.0f*sd(q0) + 2.0f*sd(qz) - 1.0f) 
        + rax*(2.0f*q0*qy + 2.0f*qx*qz) - ray*(2.0f*q0*qx - 2.0f*qy*qz));

    derivative(8) = 2.0f*(2.0f*qx*rax + 4*qy*ray + 2.0f*qz*raz)*(y - rby + ray*(2.0f*sd(q0) + 2.0f*sd(qy) - 1.0f) 
        + rax*(2.0f*q0*qz + 2.0f*qx*qy) + raz*(2.0f*q0*qx + 2.0f*qy*qz)) 
        + 2.0f*(2.0f*q0*raz - 2.0f*qx*ray)*(rbx - x - rax*(2.0f*sd(q0) + 2.0f*sd(qx) - 1.0f) 
        + ray*(2.0f*q0*qz - 2.0f*qx*qy) + raz*(2.0f*q0*qy - 2.0f*qx*qz)) 
        + 2.0f*(2.0f*q0*rax + 2.0f*qz*ray)*(z - rbz + raz*(2.0f*sd(q0) + 2.0f*sd(qz) - 1.0f) 
        + rax*(2.0f*q0*qy + 2.0f*qx*qz) - ray*(2.0f*q0*qx - 2.0f*qy*qz));

    derivative(9) = 2.0f*(2.0f*qx*rax + 2.0f*qy*ray + 4*qz*raz)*(z - rbz + raz*(2.0f*sd(q0) + 2.0f*sd(qz) - 1.0f) 
        + rax*(2.0f*q0*qy + 2.0f*qx*qz) - ray*(2.0f*q0*qx - 2.0f*qy*qz)) 
        + 2.0f*(2.0f*q0*ray - 2.0f*qx*raz)*(rbx - x - rax*(2.0f*sd(q0) + 2.0f*sd(qx) - 1.0f) 
        + ray*(2.0f*q0*qz - 2.0f*qx*qy) + raz*(2.0f*q0*qy - 2.0f*qx*qz)) 
        + 2.0f*(2.0f*q0*rax + 2.0f*qy*raz)*(y - rby + ray*(2.0f*sd(q0) + 2.0f*sd(qy) - 1.0f) 
        + rax*(2.0f*q0*qz + 2.0f*qx*qy) + raz*(2.0f*q0*qx + 2.0f*qy*qz));

    return derivative;
}

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
