#include "Visualization.h"
#include <iostream>

Visualization::Visualization() {}

void Visualization::visualize(std::vector<Robot>& Robots, std::vector<Landmark>& Landmark_Groundtruth, double timesteps, double sample_time) {
    std::cout << "Visualizing data for " << Robots.size() << " robots with " << Landmark_Groundtruth.size() << " landmarks at time " << timesteps << " with sample time " << sample_time << "..." << std::endl;
}
