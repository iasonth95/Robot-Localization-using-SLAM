// Barcode.h
#ifndef BARCODE_H
#define BARCODE_H

struct Barcode
{
    int subject_num;
    int barcode_num;
};

#endif

// Landmark_Groundtruth.h
#ifndef LANDMARK_GROUNDTRUTH_H
#define LANDMARK_GROUNDTRUTH_H

struct Landmark_Groundtruth
{
    int subject_num;
    double x;
    double y;
    double x_sd;
    double y_sd;
};

#endif

// Robot.h
#ifndef ROBOT_H
#define ROBOT_H

#include <vector> // Include any necessary headers for vector

struct Robot
{
    std::vector<double> time;        // time
    std::vector<double> x;           // position
    std::vector<double> y;           // position
    std::vector<double> theta;       // orientation
    std::vector<double> v;           // linear velocity
    std::vector<double> w;           // angular velocity
    std::vector<double> barcode_num; // barcode on each robot and landmark
    std::vector<double> r;           // range
    std::vector<double> b;           // bearing (angle)

    std::vector<double> sampled_time;
    std::vector<double> sampled_x;
    std::vector<double> sampled_y;
    std::vector<double> sampled_theta;
    std::vector<double> sampled_v;
    std::vector<double> sampled_w;
    std::vector<double> sampled_barcode_num;
    std::vector<double> sampled_r;
    std::vector<double> sampled_b;
};

#endif

#ifndef DATALOADER_H
#define DATALOADER_H

#include <vector>
#include <tuple>
// Include any other necessary headers

class DataLoader
{
public:
    DataLoader(int n_robots);
    std::tuple<std::vector<Barcode>, std::vector<Landmark_Groundtruth>, std::vector<Robot>> loadData(std::vector<Barcode> &Barcodes,
                                                                                                     std::vector<Landmark_Groundtruth> &Landmarks,
                                                                                                     std::vector<Robot> &Robots); // Add any other necessary member functions or data members

    void printData(const std::vector<Barcode> &Barcodes,
                   const std::vector<Landmark_Groundtruth> &Landmarks,
                   const std::vector<Robot> &Robots);

private:
    int n_robots_;
};

#endif