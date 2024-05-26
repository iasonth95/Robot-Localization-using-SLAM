#include "EKFAlgorithm.h"
#include <iostream>

EKFAlgorithm::EKFAlgorithm() {}

void EKFAlgorithm::run(std::vector<Robot>& Robots, std::vector<Landmark>& Landmark_Groundtruth, double timesteps, double sample_time) {
    std::cout << "Running EKF algorithm for " << Robots.size() << " robots with " << Landmark_Groundtruth.size() << " landmarks at time " << timesteps << " with sample time " << sample_time << "..." << std::endl;
}
