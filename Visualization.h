#ifndef VISUALIZATION_H
#define VISUALIZATION_H
#include <vector> // Include the necessary data types

class Visualization {
public:
    Visualization(); // Constructor
    void visualize(std::vector<Robot>& Robots, std::vector<Landmark>& Landmark_Groundtruth, double timesteps, double sample_time);
    // Define any other member functions and data structures needed
};

#endif
