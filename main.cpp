#include "DataLoader.h"
#include "Dataset.h"
#include "Observation.h"
#include "ConBear.h"
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <Eigen/Dense>
#include "Visualization.h" // Include Visualization class header

using namespace Eigen;

class RobotEstimation
{
public:
    std::vector<Pose> poseMeans;
};

double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0; // Convert degrees to radians using the formula
}

int main()
{
    double deltaT = 0.02;                                           // Sample step
    std::vector<double> alphas = {0.1, 0.01, 0.18, 0.08, 0.0, 0.0}; // Motion model noise parameters

    // Measurement model noise parameters
    double sigma_range = 3.4; // sqrt(11.8)
    double sigma_bearing = 0.42; // sqrt(0.18)
    double sigma_id = 1.0;

    // Q_t matrix
    Matrix2d Q_t;
    //Q_t << sigma_range * sigma_range, 0.0, 0.0,
        //0.0, sigma_bearing * sigma_bearing, 0.0,
        //0.0, 0.0, sigma_id * sigma_id;
    Q_t << 11.8, 0.0,
        0.0, 0.18;
    double measurement_prob = 0.0;
    int robot_num = 1;
    int n_landmarks = 15;
    int n_robots = 1;          // Set the number of robots
    double sample_time = 0.02; // Set the sampling time
    std::vector<Barcode> Barcodes;
    std::vector<Landmark_Groundtruth> Landmarks;
    std::vector<Robot> Robots;

    DataLoader dataLoader(n_robots);
    Dataset sampling_dataset;
    // Call the data loading method to load the data
    dataLoader.loadData(Barcodes, Landmarks, Robots);
    dataLoader.printData(Barcodes, Landmarks, Robots);
    auto result = sampling_dataset.sample(Robots, sample_time);
    // sampling_dataset.printSampledData(result);
    // Localization task
    robot_num = robot_num - 1; // Specify the robot number
    std::cout << "Localization task for robot " << Robots[robot_num].sampled_v[0] << std::endl;
    RobotEstimation robotEstimation;

    // Initialize Estimation
    int start = 599;                                  // Start index
    double t = Robots[robot_num].sampled_time[start]; // Start time
    Eigen::Vector3d stateMean3d;
    stateMean3d << Robots[robot_num].sampled_x[start],
                     Robots[robot_num].sampled_y[start],
                     Robots[robot_num].sampled_theta[start];

    Eigen::MatrixXd F_x(3, 3 + 2 * n_landmarks);
    F_x.leftCols(3) = Eigen::MatrixXd::Identity(3, 3);
    F_x.rightCols(2 * n_landmarks) = Eigen::MatrixXd::Zero(3, 2 * n_landmarks);
    Eigen::VectorXd stateMean = F_x.transpose() * stateMean3d;

    Eigen::MatrixXd stateCov(2 * n_landmarks + 3, 2 * n_landmarks + 3);
    stateCov.setZero();
    stateCov.block(0, 0, 3, 3).setConstant(0.001);


    for (int i = 3; i < 2 * n_landmarks + 3; ++i) {
        stateCov(i, i) = 10;
    }

    // Track which measurement is next received
    int measurementIndex = 0;

    // Set up map between barcodes and landmark IDs (known correlation)
    std::unordered_map<double, double> codeDict;
    for (const Barcode &barcode : Barcodes)
    {
        codeDict[barcode.barcode_num] = barcode.subject_num;
    }

    // Loop through all odometry and measurement samples
    // Updating the robot's pose estimate with each step
    //#pragma omp parallel for num_threads(2) // Adjust num_threads as needed
    for (int i = start; i < Robots[robot_num].sampled_time.size(); ++i)
    {
        double theta = stateMean(2); // Theta
        // Update time
        t = Robots[robot_num].sampled_time[i];
        // Update movement vector per equation 1
        std::vector<double> u_t = {Robots[robot_num].sampled_v[i], Robots[robot_num].sampled_w[i]}; // [v_t ; omega_t]
        double rot = deltaT * u_t[1];                                                               // rot = delta_theta = deltaT * omega_t
        double halfRot = rot / 2.0;
        double trans = u_t[0] * deltaT; // Translational: v_t * deltaT

        //MatrixXd G_t(2 * n_landmarks + 3, 2 * n_landmarks + 3);
        Matrix2d M_t;
        MatrixXd V_t(3, 2); // Define V_t as 3x2 matrix
        Vector3d poseUpdate;

        // Calculate the movement Jacobian per the updated equation
        // Compute g_t and G_t
        Eigen::MatrixXd g_t(3, 3);
        g_t << 0, 0, trans * -sin(theta + halfRot),
            0, 0, trans * cos(theta + halfRot),
            0, 0, 0;
        Eigen::MatrixXd G_t = Eigen::MatrixXd::Identity(2 * n_landmarks + 3, 2 * n_landmarks + 3) + F_x.transpose() * g_t * F_x;


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
            rot; // Assuming u_t(1) is the rotation component (rot)

        // Calculate estimated state mean per equation 1
        Eigen::VectorXd stateMeanBar = stateMean + F_x.transpose() * poseUpdate;

        // Constrain bearing to +/- pi
        ConBear bearing;
        stateMeanBar(2) = bearing.conBear(stateMeanBar(2));

        MatrixXd R_t = V_t * M_t * V_t.transpose();
        // Calculate estimated state covariance per equation 5 (Σ^t)
        Eigen::MatrixXd stateCovBar = G_t * stateCov * G_t.transpose() + F_x.transpose() * R_t * F_x;

        Observation get_observations;
        Eigen::MatrixXd z(3, 1); // Initialize z as a 3x1 vector

        // Get measurements for the current timestep, if any
        std::tie(z, measurementIndex) = get_observations.get(Robots, robot_num, t, measurementIndex, codeDict);
        //std::cout << "z:\n"
                          //<< z << std::endl;
                //std::cout << "poseMeanBar:\n";
        // Initialize zHat and S before processing measurements
        Eigen::MatrixXd zHat;
/*      std::cout<<"stateMeanBar"<<std::endl;
        std::cout<<stateMeanBar<<std::endl;
        std::cout << "V_t:\n"
                          << V_t << std::endl;
                std::cout << "M_t:\n"
                          << M_t<< std::endl;          
                std::cout << "G_t:\n"
                          << G_t << std::endl;
                std::cout << "poseUpdate:\n"
                          << poseUpdate << std::endl;
                std::cout << "stateMeanBar:\n"
                          << stateMeanBar << std::endl;
                std::cout << "stateCov:\n"
                          << stateCov << std::endl;
                std::cout << "stateCovBar:\n"
                          << stateCovBar << std::endl;
                std::cout<<"F_x"<<std::endl;
        std::cout<<F_x<<std::endl;
std::cout<<"R_t"<<std::endl;
        std::cout<<R_t<<std::endl;
        */  
        
        // If any measurements are available
        if (z.cols() > 0 && z(2, 0) > 1)
        {
            // Resize zHat based on number of measurements
            zHat.resize(2, z.cols());
            zHat.setZero();  // Set all elements to zero

            // Loop over every measurement
            for (int k = 0; k < z.cols(); ++k)
            {
                int j = static_cast<int>(z(2, k)); // Ensure the index is an integer

                // If the landmark has never been seen before
                //std::cout<< stateMeanBar(3 + 2 * j) << stateMeanBar(2 + 2 * j) << stateMeanBar(1 + 2 * j)  << std::endl;
                if (stateMeanBar(3 + 2 * j) == 0)
                {
                    // Add it to the state vector
                    double landmark_pos_x = z(0, k) * cos(z(1, k) + stateMeanBar(2));
                    double landmark_pos_y = z(0, k) * sin(z(1, k) + stateMeanBar(2));
                    stateMeanBar(3 + 2 * j) = stateMeanBar(0) + landmark_pos_x;
                    stateMeanBar(4 + 2 * j) = stateMeanBar(1) + landmark_pos_y;
                    continue;
                }

                double delta_x = stateMeanBar(3 + 2 * j) - stateMeanBar(0);
                double delta_y = stateMeanBar(4 + 2 * j) - stateMeanBar(1);
                double q = delta_x * delta_x + delta_y * delta_y;
                double r = std::sqrt(q); // Predicted range to landmark

                // Predicted bearing to landmark
                double pred_bear = bearing.conBear(std::atan2(delta_y, delta_x) - stateMeanBar(2));

                // Create predicted measurement zHat per equation 7
                zHat(0, k) = r;
                zHat(1, k) = pred_bear;

                // Calculate measurement Jacobian
                Eigen::MatrixXd h_t(2, 5);  // Initialize h_t as a 2x5 matrix
                h_t << -delta_x / r, -delta_y / r, 0, delta_x / r, delta_y / r,
                    delta_y / q, -delta_x / q, -1, -delta_y / q, delta_x / q;

                // Define F_1 and F_2 matrices
                Eigen::MatrixXd F_1(5, 3);  // Initialize F_1 as a 5x3 matrix
                F_1.setZero();  // Set all elements to zero
                F_1.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);

                Eigen::MatrixXd F_2(5, 2);  // Initialize F_2 as a 5x2 matrix
                F_2.setZero();  // Set all elements to zero
                F_2.block<2, 2>(3, 0) = Eigen::MatrixXd::Identity(2, 2);

                // Construct F_xj
                Eigen::MatrixXd F_xj = Eigen::MatrixXd::Zero(5, 2 * n_landmarks + 3);  // Initialize F_xj as a 5x(2*n_landmarks+3) matrix
                F_xj.block<5, 3>(0, 0) = F_1;
                F_xj.block(0, 3 + 2 * j, 5, 2) = F_2;

                // Calculate H_t
                Eigen::MatrixXd H_t = h_t * F_xj;

                // Define K_t matrix (Kalman gain)
                Eigen::MatrixXd K_t = stateCovBar * H_t.transpose() * (H_t * stateCovBar * H_t.transpose() + Q_t).inverse();  // Assuming S_vector has only one element
                //std::cout << "zhat:\n"
                            //<< zHat << std::endl;
                    //std::cout << "zHat:\n";
                
             
                //std::cout << "zhat:\n"
                            //<< z.col(k).head<2>() << std::endl;
                    //std::cout << "zHatWEIRD:\n";
                Eigen::MatrixXd innovation = z.col(k).head<2>() - zHat.col(k);

                
                // Update state mean and covariance
                stateMeanBar = stateMeanBar + K_t * innovation;
                stateCovBar = (Eigen::MatrixXd::Identity(stateCovBar.rows(), stateCovBar.cols()) - K_t * H_t) * stateCovBar;

                // Normalize the angle of the stateMean vector to be within -pi to pi
                //stateMean(2) = std::atan2(std::sin(stateMean(2)), std::cos(stateMean(2)));
            }
        }
        
        stateMean = stateMeanBar;
        //std::cout<<"stateMeanBar"<<std::endl;
        //std::cout<<stateMeanBar<<std::endl;
        stateCov = stateCovBar;
        stateMean(2) = bearing.conBear(stateMean(2));
        //std::cout << "stateMeannnn:\n"
                        //<< stateMean << std::endl;
        //std::cout << "stateCovvvvvv:\n"
                        //<< stateCov << std::endl;
        

        Pose estimatedPose;
        estimatedPose.x = stateMean(0);
        estimatedPose.y = stateMean(1);
        estimatedPose.theta = stateMean(2);
        robotEstimation.poseMeans.push_back(estimatedPose);
        
    

        // Calculate error between mean pose and ground truth for testing only
        //Vector3d groundtruth;
        //groundtruth << Robots[robot_num].x[i],
            //Robots[robot_num].y[i],
            //Robots[robot_num].theta[i];

        //Vector3d error = groundtruth - Vector3d(poseMean.x, poseMean.y, poseMean.theta);
        //std::cout << "Error between ground truth and poseMean: " << error.transpose() << std::endl;
    }


    std::cout << "Now we go to the latest loop for visualizing" << std::endl;
    // Initialize SFML visualization
    Visualization vis;
    std::cout << "Now we go to the latest loop for visualizing" << std::endl;

    // Create vectors to store the ground truth values for each timestep
    std::vector<double> groundTruthX;
    std::vector<double> groundTruthY;
    std::vector<double> groundTruthTheta;

    // Populate the ground truth vectors with the robot's sampled values
    for (int i = start; i < Robots[0].sampled_x.size(); ++i)
    {
        groundTruthX.push_back(Robots[0].sampled_x[i]);
        groundTruthY.push_back(Robots[0].sampled_y[i]);
        groundTruthTheta.push_back(Robots[0].sampled_theta[i]);
    }

    std::cout << "Now we go to the latest loop for visualizing" << std::endl;

    // Main loop to simulate continuous updating of the visualization
    for (int i = 0; i < robotEstimation.poseMeans.size(); ++i)
    {
        // Retrieve poseMean for current timestep
        std::vector<Pose> poseMeans = robotEstimation.poseMeans;

        // Draw pose and groundtruth in the visualization
        vis.drawPoseAndGroundtruth(poseMeans, groundTruthX, groundTruthY, groundTruthTheta, i);

        // Display the updated visualization
        vis.display();

        // Handle events to keep the window open until manually closed
        vis.handleEvents();

        // Optional delay to control frame rate
        // sf::sleep(sf::milliseconds(50));  // Adjust as needed
    }

    // Keep the window open until it is manually closed
    while (vis.isOpen())
    {
        vis.handleEvents();
    }

    std::cout << "Finished visualization." << std::endl;

    return 0;
}