#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <iostream>
#include "DataLoader.h"

class Dataset {
public:
    Dataset();
    std::tuple<std::vector<Robot>, int> sample(std::vector<Robot>& Robots, double sample_time);

private:
    void normalizeTime(std::vector<Robot>& Robots, double& min_time, double& max_time);
};

#endif
