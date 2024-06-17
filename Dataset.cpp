#include "Dataset.h"
#include <iostream>
#include <cmath>
#include <limits>

Dataset::Dataset() {}

void Dataset::normalizeTime(std::vector<Robot>& Robots, double& min_time, double& max_time) {
    min_time = std::numeric_limits<double>::max();
    max_time = std::numeric_limits<double>::lowest();

    for (auto& robot : Robots) {
        if (!robot.time.empty()) {
            min_time = std::min(min_time, robot.time.front());
            max_time = std::max(max_time, robot.time.back());
        }
    }

    for (auto& robot : Robots) {
        for (auto& t : robot.time) {
            t -= min_time;
        }
    }

    max_time -= min_time;
}

std::tuple<std::vector<Robot>, int> Dataset::sample(std::vector<Robot>& Robots, double sample_time) {
    std::cout << "Sampling dataset for " << Robots.size() << " robots with sample time " << sample_time << "..." << std::endl;
    int timesteps = 0;

    double min_time, max_time;
    normalizeTime(Robots, min_time, max_time);

    timesteps = std::floor(max_time / sample_time) + 1;

    for (auto& robot : Robots) {
        std::vector<double> sampled_time(timesteps);
        std::vector<double> sampled_x(timesteps);
        std::vector<double> sampled_y(timesteps);
        std::vector<double> sampled_theta(timesteps);
        std::vector<double> sampled_v(timesteps);
        std::vector<double> sampled_w(timesteps);
        std::vector<double> sampled_barcode_num(timesteps);
        std::vector<double> sampled_r(timesteps);
        std::vector<double> sampled_b(timesteps);

        int k = 0;
        double t = 0;
        size_t i = 0;
        double p = 0;

        while (t <= max_time) {
            sampled_time[k] = t;

            while (i < robot.time.size() && robot.time[i] <= t) {
                ++i;
            }

            if (i == 1 || i == robot.time.size()) {
                sampled_x[k] = 0;
                sampled_y[k] = 0;
                sampled_theta[k] = 0;
                sampled_v[k] = 0;
                sampled_w[k] = 0;
                sampled_barcode_num[k] = 0;
                sampled_r[k] = 0;
                sampled_b[k] = 0;
            } else {
                p = (t - robot.time[i - 1]) / (robot.time[i] - robot.time[i - 1]);

                sampled_x[k] = (1 - p) * robot.x[i - 1] + p * robot.x[i];
                sampled_y[k] = (1 - p) * robot.y[i - 1] + p * robot.y[i];

                double d_theta = robot.theta[i] - robot.theta[i - 1];
                if (d_theta > M_PI) {
                    d_theta -= 2 * M_PI;
                } else if (d_theta < -M_PI) {
                    d_theta += 2 * M_PI;
                }
                sampled_theta[k] = (1 - p) * robot.theta[i - 1] + p * (robot.theta[i - 1] + d_theta);

                sampled_v[k] = (1 - p) * robot.v[i - 1] + p * robot.v[i];
                sampled_w[k] = (1 - p) * robot.w[i - 1] + p * robot.w[i];
                sampled_barcode_num[k] = (1 - p) * robot.barcode_num[i - 1] + p * robot.barcode_num[i];
                sampled_r[k] = (1 - p) * robot.r[i - 1] + p * robot.r[i];
                sampled_b[k] = (1 - p) * robot.b[i - 1] + p * robot.b[i];
            }
            ++k;
            t += sample_time;
        }

        robot.sampled_time = sampled_time;
        robot.sampled_x = sampled_x;
        robot.sampled_y = sampled_y;
        robot.sampled_theta = sampled_theta;
        robot.sampled_v = sampled_v;
        robot.sampled_w = sampled_w;
        robot.sampled_barcode_num = sampled_barcode_num;
        robot.sampled_r = sampled_r;
        robot.sampled_b = sampled_b;
    }

    std::cout << "Dataset sampling completed." << std::endl;

    return std::make_tuple(Robots, timesteps);
}
