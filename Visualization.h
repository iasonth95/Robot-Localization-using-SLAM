#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <SFML/Graphics.hpp>
#include <vector>
#include "DataLoader.h"
#include <iostream>
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
                                const std::vector<Landmark_Groundtruth> &Landmarks,
                                int currentStep);

    // Display the window
    void display();

    // Check if the window is open
    bool isOpen() const;

    // Handle events (e.g., window close events)
    void handleEvents();

private:
    sf::RenderWindow window_;
    sf::Texture carTexture;  // Car texture
    sf::Texture carTexture1;
    sf::Sprite carSprite;    // Car sprite
    sf::Sprite carSprite1;    // Car sprite
    sf::Font font;
    sf::Text text;
    sf::CircleShape landmarkShape;
    float scale_;  // Zoom scale
    float scaleFactorX_;
    float scaleFactorY_;
};



#endif // VISUALIZATION_H
