#include "DataLoader.h"
#include "Dataset.h"
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <armadillo>

// Define a struct for pose mean and covariance
struct Pose {
    double x;
    double y;
    double theta;
};

// Define a struct for robot estimation
struct RobotEstimation {
    std::vector<Pose> poseMeans;
    std::vector<std::vector<double>> poseCovs;
};

// Function to perform matrix addition
std::vector<std::vector<double>> matrixAdd(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    // Implement matrix addition here
}

// Function to perform matrix multiplication
std::vector<std::vector<double>> matrixMultiply(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    // Implement matrix multiplication here
}

// Function to compute the transpose of a matrix
std::vector<std::vector<double>> transpose(const std::vector<std::vector<double>>& A) {
    // Implement matrix transpose here
}

int main() {

    double deltaT = 0.02; // Sample step
    std::vector<double> alphas = {0.2, 0.03, 0.09, 0.08, 0.0, 0.0}; // Motion model noise parameters

    // Measurement model noise parameters
    double sigma_range = 0.43;
    double sigma_bearing = 0.6;
    double sigma_id = 1.0;

    // Q_t matrix
    std::vector<std::vector<double>> Q_t = {
        {sigma_range * sigma_range, 0.0, 0.0},
        {0.0, sigma_bearing * sigma_bearing, 0.0},
        {0.0, 0.0, sigma_id * sigma_id}
    };

    double measurement_prob = 0.0;
    int robot_num = 1;
    int n_landmarks = 15;
    int n_robots = 1; // Set the number of robots
    double sample_time = 0.02; // Set the sampling time
    std::vector<Barcode> Barcodes;
    std::vector<Landmark_Groundtruth> Landmarks;
    std::vector<Robot> Robots;

    DataLoader dataLoader(n_robots);
    Dataset sampling_dataset;
    // Call the data loading method to load the data
    dataLoader.loadData(Barcodes, Landmarks, Robots);
    auto result = sampling_dataset.sample(Robots, sample_time);

    // Localization task
    robot_num = robot_num - 1; // Specify the robot number
    RobotEstimation robotEstimation;

    // Initialize Estimation
    int start = 600; // Start index
    double t = Robots[robot_num].sampled_time[start]; // Start time
    Pose poseMean = {Robots[robot_num].sampled_x[start], 
                     Robots[robot_num].sampled_y[start], 
                     Robots[robot_num].sampled_theta[start]};
    std::vector<std::vector<double>> poseCov = {{0.01, 0.01, 0.01},
                                                 {0.01, 0.01, 0.01},
                                                 {0.01, 0.01, 0.01}};

    // Track which measurement is next received
    int measurementIndex = 0;

    // Set up map between barcodes and landmark IDs (known correlation)
    std::unordered_map<double, double> codeDict;
    for (const Barcode& barcode : Barcodes) {
        codeDict[barcode.barcode_num] = barcode.subject_num;
    }

    // Check if time of measurement index is smaller than t = 600
    while (Robots[robot_num].time[measurementIndex] < t - 0.05) {
        // Update index
        ++measurementIndex;
    }
    // Loop through all odometry and measurement samples
    // Updating the robot's pose estimate with each step
    // Reference table 7.2 in Probabilistic Robotics
    // Starts from 600 for some reason (the first 599 have a problem?)
    for (int i = start; i < Robots[robot_num].time.size(); ++i) {
        double theta = poseMean.theta; // Theta
        // Update time
        double t = Robots[robot_num].time[i];
        // Update movement vector per equation 1
        std::vector<double> u_t = {Robots[robot_num].v[i], Robots[robot_num].w[i]}; // [v_t ; omega_t]
        double rot = deltaT * u_t[1]; // rot = delta_theta = deltaT * omega_t
        double halfRot = rot / 2.0; // Just half the rotational
        double trans = u_t[0] * deltaT; // Translational: v_t * deltaT 
        // DeltaT here is defined as deltaS_r = deltaS_l

        // Calculate the movement Jacobian per equation 2 (matrix A_t)
        // Trans instead of only v_t
        std::vector<std::vector<double>> G_t = {
            {1.0, 0.0, trans * -sin(theta + halfRot)},
            {0.0, 1.0, trans * cos(theta + halfRot)},
            {0.0, 0.0, 1.0}
        };

        // Calculate motion covariance in control space per equation 3 (P_t or Sigma_s)
        std::vector<std::vector<double>> M_t = {
            {pow(alphas[0] * fabs(u_t[0]) + alphas[1] * fabs(u_t[1]), 2), 0.0},
            {0.0, pow(alphas[2] * fabs(u_t[0]) + alphas[3] * fabs(u_t[1]), 2)}
        };

        // Calculate Jacobian to transform motion covariance to state space per equation 4 (This is equation G_t but for 2D)
        std::vector<std::vector<double>> V_t = {
            {cos(theta + halfRot), -0.5 * sin(theta + halfRot)},
            {sin(theta + halfRot), 0.5 * cos(theta + halfRot)},
            {0.0, 1.0}
        };

        // Calculate pose update 
        // In order to obtain x_t+1, y_t+1, theta_t+1 (Page 15 MIT slam slides)
        std::vector<double> poseUpdate = {
            trans * cos(theta + halfRot),
            trans * sin(theta + halfRot),
            rot
        };

        // Calculate estimated pose mean per equation 1
        std::vector<double> poseMeanBar = {poseMean.x, poseMean.y, poseMean.theta};
        for (int j = 0; j < 3; ++j) {
            poseMeanBar[j] += poseUpdate[j];
        }

        
        // Calculate estimated pose covariance per equation 5 (P_t+1|t)
        std::vector<std::vector<double>> poseCovBar = matrixAdd(
            matrixMultiply(matrixMultiply(G_t, poseCov), transpose(G_t)),
            matrixMultiply(matrixMultiply(V_t, M_t), transpose(V_t))
        );

        // Store estimation
        //robotEstimation.poseMeans.push_back({poseMeanBar[0], poseMeanBar[1], poseMeanBar[2]});
        //robotEstimation.poseCovs.push_back(poseCovBar);
}
    

        return 0;
    }