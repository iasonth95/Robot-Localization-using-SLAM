#ifndef EKFALGORITHM_H
#define EKFALGORITHM_H
#include <vector> // Include the necessary data types

class EKFAlgorithm {
public:
    EKFAlgorithm(); // Constructor
    std::vector<Robot> run(std::vector<Robot>& Robots, std::vector<Landmark>& Landmark_Groundtruth, double timesteps, double sample_time); // Return a vector of updated Robots
    // Define any other member functions and data structures needed
};

#endif
