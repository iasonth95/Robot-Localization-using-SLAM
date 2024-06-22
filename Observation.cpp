#include "Observation.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>

Observation::Observation()
{
    // Constructor implementation (if needed)
}

std::tuple<Eigen::MatrixXd, int> Observation::get(std::vector<Robot> &Robots, int robot_num, double t, int &index, std::unordered_map<double, double> &codeDict)
{
    Eigen::MatrixXd z = Eigen::MatrixXd::Zero(3, 1); // Initialize z as a 3x1 zero vector

    // Ensure the index is within bounds
    while (index < Robots[robot_num].barcode_num.size() && Robots[robot_num].measurement_time[index] - t < 0.005)
    {
        double barcode = Robots[robot_num].barcode_num[index];
        double landmarkID = 0;

        // Check if the barcode exists in the codeDict
        if (codeDict.find(barcode) != codeDict.end())
        {
            landmarkID = codeDict[barcode];
        }
        else
        {
            std::cerr << "Key not found" << std::endl;
        }

        if (landmarkID > 5 && landmarkID < 21)
        {
            double range = Robots[robot_num].r[index];   // r
            double bearing = Robots[robot_num].b[index]; // phi or alpha

            // Update z with the current observation
            z << range, bearing, landmarkID - 6;
        }
        index++;
    }

    return std::make_tuple(z, index);
}
