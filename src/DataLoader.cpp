#include "DataLoader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <tuple>

DataLoader::DataLoader(int n_robots) : n_robots_(n_robots) {}

std::tuple<std::vector<Barcode>, std::vector<Landmark_Groundtruth>, std::vector<Robot>> DataLoader::loadData(
    std::vector<Barcode> &Barcodes,
    std::vector<Landmark_Groundtruth> &Landmarks,
    std::vector<Robot> &Robots)
{
    // Load Barcodes data
    std::ifstream barcodeFile("./dataset_UTIAS/Barcodes.dat");
    if (barcodeFile.is_open())
    {
        std::cout << "Dataset sampling completed." << std::endl;
        Barcode barcode;
        std::string line;
        while (std::getline(barcodeFile, line))
        {
            // Skip lines starting with "#" or empty lines
            if (line.empty() || line[0] == '#' || line.find("UTIAS") != std::string::npos || line.find("produced by") != std::string::npos)
            {
                continue;
            }
            std::istringstream iss(line); // Declare iss here
            if (iss >> barcode.subject_num >> barcode.barcode_num)
            {
                // std::cout << barcode.subject_num << " " << barcode.barcode_num << std::endl << "hello" << std::endl;
                Barcodes.push_back(barcode);
            }
            else
            {
                // Handle parsing error if needed
                std::cerr << "Error parsing line: " << line << std::endl;
            }
        }
        barcodeFile.close();
    }
    else
    {
        std::cerr << "Error opening Barcodes.dat" << std::endl;
    }

    // Load Landmark_Groundtruth data
    std::ifstream landmarkFile("./dataset_UTIAS/Landmark_Groundtruth.dat");
    if (landmarkFile.is_open())
    {
        Landmark_Groundtruth landmark;
        std::string line;
        while (std::getline(landmarkFile, line))
        {
            // Skip lines starting with "#" or empty lines
            if (line.empty() || line[0] == '#' || line.find("UTIAS") != std::string::npos || line.find("produced by") != std::string::npos)
            {
                continue;
            }
            std::istringstream iss(line); // Declare iss here
            if (iss >> landmark.subject_num >> landmark.x >> landmark.y >> landmark.x_sd >> landmark.y_sd)
            {
                // std::cout << barcode.subject_num << " " << barcode.barcode_num << std::endl << "hello" << std::endl;
                Landmarks.push_back(landmark);
            }
            else
            {
                // Handle parsing error if needed
                std::cerr << "Error parsing line: " << line << std::endl;
            }
        }
        barcodeFile.close();
    }
    else
    {
        std::cerr << "Error opening Barcodes.dat" << std::endl;
    }

    // Load data for each robot
    for (int i = 1; i <= n_robots_; ++i)
    {
        Robot robot;
        std::ifstream robotFile("./dataset_UTIAS/Robot" + std::to_string(i) + "_Groundtruth.dat");
        if (robotFile.is_open())
        {
            std::string line;
            while (std::getline(robotFile, line))
            {
                std::istringstream iss(line);
                double time, x, y, theta;
                if (iss >> time >> x >> y >> theta)
                {
                    robot.time.push_back(time);
                    robot.x.push_back(x);
                    robot.y.push_back(y);
                    robot.theta.push_back(theta);
                }
                else
                {
                    // Handle parsing error if needed
                    std::cerr << "Error parsing line: " << line << std::endl;
                }
            }
        }
        else
        {
            std::cerr << "Error opening Robot" << i << "_Groundtruth.dat" << std::endl;
        }

        robotFile.close();
        // Load odometry data
        std::ifstream odometryFile("./dataset_UTIAS/Robot" + std::to_string(i) + "_Odometry.dat");
        if (odometryFile.is_open())
        {
            std::string line;
            while (std::getline(odometryFile, line))
            {
                std::istringstream iss(line);
                double time, v, w;
                if (iss >> time >> v >> w)
                {
                    robot.v.push_back(v);
                    robot.w.push_back(w);
                }
                else
                {
                    // Handle parsing error if needed
                    std::cerr << "Error parsing line: " << line << std::endl;
                }
            }
        }
        else
        {
            std::cerr << "Error opening Robot" << i << "_Odometry.dat" << std::endl;
        }
        odometryFile.close();

        // Construct file name
        std::ifstream measurmentFile("./dataset_UTIAS/Robot" + std::to_string(i) + "_Measurement.dat");
        if (measurmentFile.is_open())
        {
            std::string line;
            while (std::getline(measurmentFile, line))
            {
                std::istringstream iss(line);
                double time, barcode_num, r, b;
                if (iss >> time >> barcode_num >> r >> b)
                {
                    robot.barcode_num.push_back(barcode_num);
                    robot.r.push_back(r);
                    robot.b.push_back(b);
                }
                else
                {
                    // Handle parsing error if needed
                    std::cerr << "Error parsing line: " << line << std::endl;
                }
            }
        }
        else
        {
            std::cerr << "Error opening Robot" << i << "_Odometry.dat" << std::endl;
        }
        measurmentFile.close();
        Robots.push_back(robot);
    }

    return std::make_tuple(Barcodes, Landmarks, Robots);
}
void DataLoader::printData(const std::vector<Barcode> &Barcodes,
                           const std::vector<Landmark_Groundtruth> &Landmarks,
                           const std::vector<Robot> &Robots)
{
    // Print Barcodes
    std::cout << "Barcodes:" << std::endl;
    for (const auto &barcode : Barcodes)
    {
        std::cout << "Subject Number: " << barcode.subject_num << ", Barcode Number: " << barcode.barcode_num << std::endl;
    }

    // Print Landmarks
    std::cout << "\nLandmarks:" << std::endl;
    for (const auto &landmark : Landmarks)
    {
        std::cout << "Subject Number: " << landmark.subject_num
                  << ", X: " << landmark.x
                  << ", Y: " << landmark.y
                  << ", X Standard Deviation: " << landmark.x_sd
                  << ", Y Standard Deviation: " << landmark.y_sd << std::endl;
    }

    // Print Robots
    std::cout << "\nRobots:" << std::endl;
    for (const auto &robot : Robots)
    {
        std::cout << "Time: ";
        for (const auto &time : robot.time)
        {
            std::cout << time << " ";
        }
        std::cout << std::endl;

        std::cout << "X: ";
        for (const auto &x : robot.x)
        {
            std::cout << x << " ";
        }
        std::cout << std::endl;

        std::cout << "Y: ";
        for (const auto &y : robot.y)
        {
            std::cout << y << " ";
        }
        std::cout << std::endl;

        std::cout << "Theta: ";
        for (const auto &theta : robot.theta)
        {
            std::cout << theta << " ";
        }
        std::cout << std::endl;
    }
}
