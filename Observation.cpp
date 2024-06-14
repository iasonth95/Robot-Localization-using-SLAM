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
    Eigen::MatrixXd z; // To store observations    std::vector<Eigen::Vector3d> observations;  // To store observations

    // Ensure the index is within bounds
    while (index < Robots[robot_num].barcode_num.size() && Robots[robot_num].time[index] - t < 0.005)
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

            if (z.cols() == 0)
            {
                z.resize(3, 1);
                z << range, bearing, landmarkID - 5;
            }
            else
            {
                Eigen::MatrixXd newZ(3, z.cols() + 1);
                newZ.block(0, 0, 3, z.cols()) = z;
                newZ.col(z.cols()) << range, bearing, landmarkID - 5;
                z = newZ;
            }
        }
        index++;
    }

    return std::make_tuple(z, index);
}
