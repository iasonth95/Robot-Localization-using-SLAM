#include "DataLoader.h"
#include "Dataset.h"
#include "Observation.h"
#include "ConBear.h"
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <armadillo>
#include <Eigen/Dense>

using namespace Eigen;

// Define a struct for pose mean and covariance
struct Pose {
    double x, y, theta;
};

class RobotEstimation {
public:
    std::vector<Pose> poseMeans;
    std::vector<Matrix3d> poseCovs;
};

double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180.0; // Convert degrees to radians using the formula
}

int main() {

    double deltaT = 0.02; // Sample step
    std::vector<double> alphas = {0.2, 0.03, 0.09, 0.08, 0.0, 0.0}; // Motion model noise parameters

    // Measurement model noise parameters
    double sigma_range = 0.43;
    double sigma_bearing = 0.6;
    double sigma_id = 1.0;

    // Q_t matrix
    Matrix3d Q_t;
    Q_t << sigma_range * sigma_range, 0.0, 0.0,
           0.0, sigma_bearing * sigma_bearing, 0.0,
           0.0, 0.0, sigma_id * sigma_id;

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
    std::cout << "Localization task for robot " << Robots[robot_num].sampled_v[0] << std::endl;
    RobotEstimation robotEstimation;

    // Initialize Estimation
    int start = 600; // Start index
    double t = Robots[robot_num].sampled_time[start]; // Start time
    Pose poseMean = {Robots[robot_num].sampled_x[start], 
                     Robots[robot_num].sampled_y[start], 
                     Robots[robot_num].sampled_theta[start]};

    Matrix3d poseCov;
    poseCov << 0.01, 0.01, 0.01,
               0.01, 0.01, 0.01,
               0.01, 0.01, 0.01;

    // Track which measurement is next received
    int measurementIndex = 0;

    // Set up map between barcodes and landmark IDs (known correlation)
    std::unordered_map<double, double> codeDict;
    for (const Barcode& barcode : Barcodes) {
        codeDict[barcode.barcode_num] = barcode.subject_num;
    }

    // Check if time of measurement index is smaller than t = 600
    //while (measurementIndex < Robots[robot_num].time.size() && Robots[robot_num].time[measurementIndex] < t - 0.05) {
    while (Robots[robot_num].time[measurementIndex] < t - 0.05) {
        // Update index
        ++measurementIndex;
    }
    // Loop through all odometry and measurement samples
    // Updating the robot's pose estimate with each step
    // Reference table 7.2 in Probabilistic Robotics
    // Starts from 600 for some reason (the first 599 have a problem?)
    for (int i = start; i < Robots[robot_num].sampled_time.size(); ++i) {
        double theta = poseMean.theta; // Theta
        //double theta_rads = degrees_to_radians(theta);
        // Update time
        double t = Robots[robot_num].sampled_time[i];
        // Update movement vector per equation 1
        std::vector<double> u_t = {Robots[robot_num].sampled_v[i], Robots[robot_num].sampled_w[i]}; // [v_t ; omega_t]
        double rot = deltaT * u_t[1]; // rot = delta_theta = deltaT * omega_t
        double halfRot = rot / 2.0;;
        /*if (rot == 0) {
            double halfRot = 0;
        }
        else{
            double halfRot = rot / 2.0;
        } // Just half the rotational}
        */
        double trans = u_t[0] * deltaT; // Translational: v_t * deltaT 
        // DeltaT here is defined as deltaS_r = deltaS_l

    Matrix3d G_t;
    Matrix2d M_t;
    MatrixXd V_t(3, 2);  // Define V_t as 3x2 matrix
    Vector3d poseUpdate;

        // Calculate the movement Jacobian per equation 2
    G_t << 1, 0, trans * -sin(theta + halfRot),
           0, 1, trans * cos(theta + halfRot),
           0, 0, 1;

    // Calculate motion covariance in control space per equation 3
    M_t << std::pow(alphas[0] * std::abs(u_t[0]) + alphas[1] * std::abs(u_t[1]), 2), 0,
       0, std::pow(alphas[2] * std::abs(u_t[0]) + alphas[3] * std::abs(u_t[1]), 2);

    // Calculate Jacobian to transform motion covariance to state space per equation 4
    V_t << cos(theta + halfRot), -0.5 * sin(theta + halfRot),
           sin(theta + halfRot), 0.5 * cos(theta + halfRot),
           0, 1;


    // Calculate pose update
    poseUpdate << trans * cos(theta + halfRot),
                  trans * sin(theta + halfRot),
                  u_t[1];  // Assuming u_t(1) is the rotation component (rot)

    // Calculate estimated pose mean per equation 1
    Vector3d poseMeanBar = {poseMean.x + poseUpdate[0],
                            poseMean.y + poseUpdate[1],
                            poseMean.theta + poseUpdate[2]};

    // Calculate estimated pose covariance per equation 5 (Σ^t)
    Matrix3d poseCovBar = G_t * poseCov * G_t.transpose() + V_t * M_t * V_t.transpose();

    // Store estimation
    robotEstimation.poseMeans.push_back({poseMeanBar[0], poseMeanBar[1], poseMeanBar[2]});
    poseCov = poseCovBar;
    //std::cout<<"Pose Covariance: "<<poseCovBar<<std::endl;
    
    Observation get_observations;

    Eigen::MatrixXd z;  // Initialize as an empty matrix
    // Get measurements for the current timestep, if any 

    // Get measurements for the current timestep, if any 
    std::tie(z, measurementIndex) = get_observations.get(Robots, robot_num, t, measurementIndex, codeDict);

    
    // Create matrices for expected measurement and measurement covariance
    Eigen::MatrixXd S;
    Eigen::MatrixXd zHat;

    // If z is not empty, update the size of S and zHat accordingly
    if (z.cols() > 0) {
        S.resize(z.cols(), 3);
        zHat.resize(3, z.cols());
    } else {
        // If z is empty, initialize S and zHat with appropriate dimensions
        // For example, you can initialize them with zeros
        S.resize(3, 3);
        S.setZero();
        zHat.resize(3, 1);  // Initialize with 1 column
        zHat.setZero();
    }

    // If any measurements are available
    if (z.cols() > 0 && z(2, 0) > 1) {
        // Loop over every measurement
        for (int k = 0; k < z.cols(); ++k) {
            int j = z(2, k);

            // Get coordinates of the measured landmark
            Vector2d m(Landmarks[j].x, Landmarks[j].y);

            // Compute the expected measurement per equations 6 and 7
            double xDist = m(0) - poseMeanBar(0); // m_j,x − µ^{hat}_t,x
            double yDist = m(1) - poseMeanBar(1);
            double q = xDist * xDist + yDist * yDist;

            ConBear bearing;
            // Constrain expected bearing to between 0 and 2*pi
            double pred_bear = bearing.conBear(atan2(yDist, xDist) - poseMeanBar(2));

            // z_{hat}_t+1|t
            zHat.col(k) << sqrt(q), pred_bear, j;

            // Calculate Jacobian of the measurement model per equation 8
            MatrixXd H(3, 3);
            H << (-1 * (xDist / sqrt(q))), (-1 * (yDist / sqrt(q))), 0,
                 (yDist / q), (-1 * (xDist / q)), -1,
                 0, 0, 0;

            //////////////////////////////////////////////////////////
            // Access element of the MatrixXd matrices
            double value = H(0, 1);
            std::cout << "Matrix:\n" << H << std::endl;
            std::cout << "Matrix:\n" << poseCovBar << std::endl;
            std::cout << "Matrix:\n" << H.transpose() << std::endl;
            std::cout << "Matrix:\n" << Q_t << std::endl;
            std::cout << "Matrix:\n" << S << std::endl;
            std::cout << "Matrix:\n" << z << std::endl;
            // Compute the expression H * poseCovBar * H.transpose() + Q_t
            Eigen::MatrixXd expression = H * poseCovBar * H.transpose() + Q_t;
            // Ensure the expression has the correct size
            std::cout << "expression:\n" << expression << std::endl;
            //assert(expression.rows() == 3 && expression.cols() == 3);
            
            // Compute S per equation 9 (just the parenthesis of Kalman gain)
            S = H * poseCovBar * H.transpose() + Q_t;

            // Compute Kalman gain per equation 10
            MatrixXd K = poseCovBar * H.transpose() * S.inverse();

            // Update pose mean and covariance estimates
            // per equations 11 and 12 (z_t+1 = z_t|t-1)
            poseMeanBar += K * (z.col(k) - zHat.col(k));
            poseCovBar = (Matrix3d::Identity() - (K * H)) * poseCovBar;
            std::cout << "poseMeanBar\n " << poseMeanBar << std::endl;
            std::cout << "poseCovBar\n " << poseCovBar << std::endl;
    }
    }
    }
    std::cout << "Finised for now the main\n " << std::endl;
        return 0;
    
}
    