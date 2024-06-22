#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <iostream>
#include <tuple>
#include "DataLoader.h"

class Dataset
{
public:
    Dataset();
    std::tuple<std::vector<Robot>, int> sample(std::vector<Robot> &Robots, double sample_time);
    void printSampledData(const std::tuple<std::vector<Robot>, int> &sampledData) const;

private:
    void normalizeTime(std::vector<Robot> &Robots, double &min_time, double &max_time);
};

#endif // DATASET_H
