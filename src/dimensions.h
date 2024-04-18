#pragma once
#include <eigen3/Eigen/Dense>

struct DistanceConstraint {
    Eigen::Vector3f stationary_joint;
    Eigen::Vector3f moving_joint;
    float distance;
};

const DistanceConstraint distance_constraints[] = {
    DistanceConstraint {
        .stationary_joint = {0.2f, -0.2f, -0.165f},
        .moving_joint = {0.2f, -0.2f, 0.0f},
        .distance = 0.275f
    },
    DistanceConstraint {
        .stationary_joint = {0.2f, 0.2f, -0.165f},
        .moving_joint = {0.2f, 0.2f, 0.0f},
        .distance = 0.275f
    },
    DistanceConstraint {
        .stationary_joint = {-0.2f, -0.2f, -0.165f},
        .moving_joint = {-0.2f, -0.2f, 0.0f},
        .distance = 0.275f
    },
    DistanceConstraint {
        .stationary_joint = {-0.2f, 0.2f, -0.165f},
        .moving_joint = {-0.2f, 0.2f, 0.0f},
        .distance = 0.275f
    },
    DistanceConstraint {
        .stationary_joint = {0.0f, 0.2f, 0.165f},
        .moving_joint = {0.0f, 0.2f, 0.0f},
        .distance = 0.275f
    },
    DistanceConstraint {
        .stationary_joint = {0.0f, -0.2f, 0.165f},
        .moving_joint = {0.0f, -0.2f, 0.0f},
        .distance = 0.275f
    }
    
    // Example constraints for testing
    // DistanceConstraint {
    //     .stationary_joint = {1.0f, 0.0f, 0.0f},
    //     .moving_joint = {0.0f, 0.0f, 0.0f},
    //     .distance = 1.0f
    // },
    // DistanceConstraint {
    //     .stationary_joint = {0.0f, 1.0f, 0.0f},
    //     .moving_joint = {0.0f, 0.0f, 0.0f},
    //     .distance = 1.0f
    // },
    // DistanceConstraint {
    //     .stationary_joint = {0.0f, 0.0f, 1.0f},
    //     .moving_joint = {0.0f, 0.0f, 0.0f},
    //     .distance = 1.0f
    // },
    // DistanceConstraint {
    //     .stationary_joint = {1.0f, 1.0f, 1.0f},
    //     .moving_joint = {0.0f, 0.0f, 0.0f},
    //     .distance = 1.0f
    // },
};

constexpr int num_constraints = static_cast<int>(sizeof(distance_constraints) / sizeof(DistanceConstraint));