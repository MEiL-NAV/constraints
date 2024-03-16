#include "constraints.h"
#include "dimensions.h"

inline float sd(float val)
{
    return val * val;
}

float distance_constraint(const Eigen::Vector<float, 13> &state,
                          const DistanceConstraint& constraint) 
{
    const float& x = state(0);
    const float& y = state(1);
    const float& z = state(2);
    const float& q0 = state(6);
    const float& qx = state(7);
    const float& qy = state(8);
    const float& qz = state(9);

    const float& rax = constraint.moving_joint(0);
    const float& ray = constraint.moving_joint(1);
    const float& raz = constraint.moving_joint(2);

    const float& rbx = constraint.stationary_joint(0);
    const float& rby = constraint.stationary_joint(1);
    const float& rbz = constraint.stationary_joint(2);

    return sd(rbx - x - rax*(2.0f * sd(q0) + 2.0f * sd(qx) - 1.0f) 
        + ray*(2.0f*q0*qz - 2.0f*qx*qy) 
        + raz*(2.0f*q0*qy - 2.0f*qx*qz)) 
        + sd(y - rby + ray*(2.0f*sd(q0) + 2.0f*sd(qy) - 1.0f) 
        + rax*(2*q0*qz + 2.0f*qx*qy) + raz*(2*q0*qx + 2.0f*qy*qz))
        + sd(z - rbz + raz*(2.0f*sd(q0) + 2.0f*sd(qz) - 1.0f) 
        + rax*(2.0f*q0*qy + 2.0f*qx*qz) - ray*(2.0f*q0*qx - 2.0f*qy*qz)) 
        - constraint.distance * constraint.distance;
}

Eigen::VectorXf distance_constraint_derivative(const Eigen::Vector<float, 13> &state,
                                               const DistanceConstraint& constraint) 
{
    Eigen::VectorXf derivative = Eigen::VectorXf::Zero(13);
    const float& x = state(0);
    const float& y = state(1);
    const float& z = state(2);
    const float& q0 = state(6);
    const float& qx = state(7);
    const float& qy = state(8);
    const float& qz = state(9);

    const float& rax = constraint.moving_joint(0);
    const float& ray = constraint.moving_joint(1);
    const float& raz = constraint.moving_joint(2);

    const float& rbx = constraint.stationary_joint(0);
    const float& rby = constraint.stationary_joint(1);
    const float& rbz = constraint.stationary_joint(2);

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
    Eigen::VectorXf FI = Eigen::VectorXf(1 + num_constraints);
    auto q = state.segment<4>(6);
    FI(0) = q.squaredNorm() - 1.0f;
    for(int i = 0; i < num_constraints; i++) {
        FI(i + 1) += distance_constraint(state, distance_constraints[i]);
    }
    return FI;
}

Eigen::MatrixXf constraints_derivative(const Eigen::Vector<float, 13> &state) {
    Eigen::MatrixXf FId = Eigen::MatrixXf::Zero(1 + num_constraints,13);
    auto q = state.segment<4>(6);
    FId.block<1,4>(0,6) = 2.0f * q.transpose();
    for(int i = 0; i < num_constraints; i++) {
        FId.row(i + 1) = distance_constraint_derivative(state, distance_constraints[i]).transpose();
    }
    return FId;
}
