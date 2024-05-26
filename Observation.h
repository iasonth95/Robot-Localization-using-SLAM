#ifndef OBSERVATION_H
#define OBSERVATION_H
#include <vector> // Include the necessary data types

class Observation {
public:
    Observation(); // Constructor
    void get(std::vector<Robot>& Robots, int robot_num, double t, int& index, std::map<int, std::string>& codeDict); // Pass parameters
    // Define any other member functions and data structures needed
};

#endif