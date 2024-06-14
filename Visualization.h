#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <SFML/Graphics.hpp>
#include <vector>
// Define a struct for pose
struct Pose {
    double x, y, theta;
};

class Visualization {
public:
    Visualization();

    void drawPoseAndGroundtruth(const std::vector<Pose>& poseMeans,
                                const std::vector<double>& sampled_x,
                                const std::vector<double>& sampled_y,
                                const std::vector<double>& sampled_theta,
                                int currentStep);
    
    void display();
    
    bool isOpen() const;
    
    void handleEvents();

private:
    sf::RenderWindow window_;
    sf::View view_;
};

#endif // VISUALIZATION_H
