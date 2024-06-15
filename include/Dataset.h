#ifndef DATASET_H
#define DATASET_H

#include <vector> // Include the necessary data types
#include <iostream>
#include "DataLoader.h"

class Dataset {
public:
    Dataset(); // Constructor


    // Declare the sample function with the appropriate parameters and return type
    std::tuple<std::vector<Robot>, int> sample(std::vector<Robot>& Robots, double sample_time);

private:
    // Add any private member variables or helper functions here
};

#endif




