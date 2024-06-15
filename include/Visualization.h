#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
// Define a struct for pose
struct Pose {
    double x, y, theta;
};

class Visualization {
public:
    Visualization();  // Constructor

    // Draw pose and ground truth
    void drawPoseAndGroundtruth(const std::vector<Pose>& poseMeans,
                                const std::vector<double>& sampled_x,
                                const std::vector<double>& sampled_y,
                                const std::vector<double>& sampled_theta,
                                int currentStep);

    // Display the window
    void display();

    // Check if the window is open
    bool isOpen() const;

    // Handle events (e.g., window close events)
    void handleEvents();

private:
    sf::RenderWindow window_;
    float scale_;  // Zoom scale
    float scaleFactorX_;
    float scaleFactorY_;
};
