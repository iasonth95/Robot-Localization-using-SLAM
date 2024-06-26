#include "Dataset.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <vector>

Dataset::Dataset() {}

void Dataset::normalizeTime(std::vector<Robot> &Robots, double &min_time, double &max_time)
{
    min_time = std::numeric_limits<double>::max();
    max_time = std::numeric_limits<double>::lowest();

    for (auto &robot : Robots)
    {
        if (!robot.time.empty())
        {
            min_time = std::min(min_time, robot.time.front());
            max_time = std::max(max_time, robot.time.back());
        }
    }

    for (auto &robot : Robots)
    {
        for (auto &t : robot.time)
        {
            t -= min_time;
        }
    }
    for (auto &robot : Robots)
    {
        for (auto &t : robot.measurement_time)
        {
            t -= min_time;
        }
    }

    for (auto &robot : Robots)
    {
        for (auto &t : robot.odometry_time)
        {
            t -= min_time;
        }
    }

    max_time -= min_time;
}

std::tuple<std::vector<Robot>, int> Dataset::sample(std::vector<Robot> &Robots, double sample_time)
{
    std::cout << "Sampling dataset for " << Robots.size() << " robots with sample time " << sample_time << "..." << std::endl;
    int timesteps = 0;

    double min_time, max_time;
    normalizeTime(Robots, min_time, max_time);

    timesteps = std::floor(max_time / sample_time) + 1;

    for (auto &robot : Robots)
    {
        std::vector<double> sampled_time(timesteps);
        std::vector<double> sampled_x(timesteps, 0.0);
        std::vector<double> sampled_y(timesteps, 0.0);
        std::vector<double> sampled_theta(timesteps, 0.0);
        std::vector<double> sampled_v(timesteps, 0.0);
        std::vector<double> sampled_w(timesteps, 0.0);
        std::vector<double> sampled_measurement_time(robot.measurement_time.size());
        std::vector<double> sampled_barcode_num(robot.barcode_num); // Identical to original
        std::vector<double> sampled_r(robot.r);                     // Identical to original
        std::vector<double> sampled_b(robot.b);                     // Identical to original

        // Sample ground truth data
        {
            int k = 0;
            double t = 0;
            size_t i = 1;

            while (t <= max_time)
            {
                sampled_time[k] = t;

                while (i < robot.time.size() && robot.time[i] <= t)
                {
                    ++i;
                }

                if (i == 1 || i == robot.time.size())
                {
                    sampled_x[k] = (i == 1) ? robot.x[0] : robot.x.back();
                    sampled_y[k] = (i == 1) ? robot.y[0] : robot.y.back();
                    sampled_theta[k] = (i == 1) ? robot.theta[0] : robot.theta.back();
                }
                else
                {
                    double p = (t - robot.time[i - 1]) / (robot.time[i] - robot.time[i - 1]);

                    sampled_x[k] = p * (robot.x[i] - robot.x[i - 1]) + robot.x[i - 1];
                    sampled_y[k] = p * (robot.y[i] - robot.y[i - 1]) + robot.y[i - 1];
                    sampled_theta[k] = p * (robot.theta[i] - robot.theta[i - 1]) + robot.theta[i - 1];

                    double d_theta = robot.theta[i] - robot.theta[i - 1];
                    if (d_theta > M_PI)
                    {
                        d_theta -= 2 * M_PI;
                    }
                    else if (d_theta < -M_PI)
                    {
                        d_theta += 2 * M_PI;
                    }
                    //sampled_theta[k] = p * d_theta + robot.theta[i - 1];
                }

                ++k;
                t += sample_time;
            }

            robot.sampled_time = sampled_time;
            robot.sampled_x = sampled_x;
            robot.sampled_y = sampled_y;
            robot.sampled_theta = sampled_theta;
        }

        // Sample odometry data
        {
            std::vector<double> sampled_odometry_time(timesteps);
            int k = 0;
            double t = 0;
            size_t i = 1;

            while (t <= max_time)
            {
                sampled_odometry_time[k] = t;

                while (i < robot.odometry_time.size() && robot.odometry_time[i] <= t)
                {
                    ++i;
                }

                if (i == 1 || i == robot.odometry_time.size())
                {
                    sampled_v[k] = (i == 1) ? robot.v[0] : robot.v.back();
                    sampled_w[k] = (i == 1) ? robot.w[0] : robot.w.back();
                }
                else
                {
                    double p = (t - robot.odometry_time[i - 1]) / (robot.odometry_time[i] - robot.odometry_time[i - 1]);

                    sampled_v[k] = p * (robot.v[i] - robot.v[i - 1]) + robot.v[i - 1];
                    sampled_w[k] = p * (robot.w[i] - robot.w[i - 1]) + robot.w[i - 1];
                }

                ++k;
                t += sample_time;
            }

            robot.sampled_v = sampled_v;
            robot.sampled_w = sampled_w;
        }

        // Adjust measurement_time according to sample_time
        for (size_t i = 0; i < robot.measurement_time.size(); ++i)
        {
            sampled_measurement_time[i] = std::floor(robot.measurement_time[i] / sample_time + 0.5) * sample_time;
        }

        robot.sampled_measurement_time = sampled_measurement_time;
        robot.sampled_barcode_num = sampled_barcode_num;
        robot.sampled_r = sampled_r;
        robot.sampled_b = sampled_b;
    }

    std::cout << "Dataset sampling completed." << std::endl;

    return std::make_tuple(Robots, timesteps);
}

void Dataset::printSampledData(const std::tuple<std::vector<Robot>, int>& sampledData) const {
    const auto& [robots, timesteps] = sampledData;

    std::cout << "Number of robots: " << robots.size() << "\n";
    std::cout << "Number of timesteps: " << timesteps << "\n";

    for (size_t i = 0; i < robots.size(); ++i) {
        const auto& robot = robots[i];
        std::cout << "\nRobot " << i + 1 << ":\n";

        std::cout << "Time (size " << robot.sampled_time.size() << "): ";
        std::cout << "Time: ";
        for (const auto& t : robot.sampled_time) {
            std::cout << t << " ";
        }
        std::cout << "\n";

        std::cout << "X (size " << robot.sampled_x.size() << "): ";
        std::cout << "X: ";
        for (const auto& x : robot.sampled_x) {
            std::cout << x << " ";
        }
        std::cout << "\n";
        std::cout << "Y (size " << robot.sampled_y.size() << "): ";
        std::cout << "Y: ";
        for (const auto& y : robot.sampled_y) {
            std::cout << y << " ";
        }
        std::cout << "\n";

        std::cout << "Theta (size " << robot.sampled_theta.size() << "): ";
        std::cout << "Theta: ";
        for (const auto& theta : robot.sampled_theta) {
            std::cout << theta << " ";
        }
        std::cout << "\n";

        std::cout << "odometry_time (size " << robot.odometry_time.size() << "): ";
        std::cout << "odometry_time: ";
        for (const auto& t : robot.odometry_time) {
            std::cout << t << " ";
        }
        std::cout << "\n";

        std::cout << "V (size " << robot.sampled_v.size() << "): ";
        std::cout << "V: ";
        for (const auto& v : robot.sampled_v) {
            std::cout << v << " ";
        }
        std::cout << "\n";

        std::cout << "W (size " << robot.sampled_w.size() << "): "; 
        std::cout << "W: ";
        for (const auto& w : robot.sampled_w) {
            std::cout << w << " ";
        }
        std::cout << "\n";
        
        std::cout << "measurement_time (size " << robot.sampled_measurement_time.size() << "): ";
        std::cout << "measurement_time: ";
        for (const auto& t : robot.sampled_measurement_time) {
            std::cout << t << " ";
        }
        std::cout << "\n";
       
        std::cout << "Barcode Num (size " << robot.sampled_barcode_num.size() << "): ";
        std::cout << "Barcode Num: ";
        for (const auto& barcode_num : robot.sampled_barcode_num) {
            std::cout << barcode_num << " ";
        }
        std::cout << "\n";

        std::cout << "R (size " << robot.sampled_r.size() << "): ";
        std::cout << "R: ";
        for (const auto& r : robot.sampled_r) {
            std::cout << r << " ";
        }
        std::cout << "\n";
    
        std::cout << "B (size " << robot.sampled_b.size() << "): ";
        std::cout << "B: ";
        for (const auto& b : robot.sampled_b) {
            std::cout << b << " ";
        }
        std::cout << "\n";
    }
}