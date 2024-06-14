#ifndef OBSERVATION_H
#define OBSERVATION_H

#include "DataLoader.h"
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

class Observation
{
public:
    Observation(); // Constructor
    std::tuple<Eigen::MatrixXd, int> get(std::vector<Robot> &Robots, int robot_num, double t, int &index, std::unordered_map<double, double> &codeDict);
};

#endif // OBSERVATION_H