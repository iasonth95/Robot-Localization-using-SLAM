#include "Visualization.h"
#include <cmath>
// Constructor: Initialize SFML window with a size and title
Visualization::Visualization() : window_(sf::VideoMode(800, 600), "Robot Localization Visualization") {
    
    
    // Set the initial view
    sf::View view(sf::FloatRect(-500.f, -500.f, 1000.f, 1000.f)); // Adjust the view size and position
    window_.setView(view);

    // Set scaling factors (adjust according to your actual units and window size)
    scaleFactorX_ = 50.f; // 1 meter in real world = 50 pixels in SFML (adjust as needed)
    scaleFactorY_ = 50.f; // 1 meter in real world = 50 pixels in SFML (adjust as needed)

    if (!carTexture.loadFromFile("carnbg.png"))
    {
        std::cout << "Error loading car image." << std::endl;
    }
    if (!carTexture1.loadFromFile("carnbgnoir.png"))
    {
        std::cout << "Error loading car image." << std::endl;
    }
    carSprite.setTexture(carTexture);
    carSprite.setOrigin(carTexture.getSize().x / 4, carTexture.getSize().y / 4);

    carSprite1.setTexture(carTexture1);
    carSprite1.setOrigin(carTexture.getSize().x / 4, carTexture.getSize().y / 4);

    if (!font.loadFromFile("arial.ttf"))
    {
        std::cout << "Error loading font." << std::endl;
    }
    text.setFont(font);
    text.setCharacterSize(15);
    text.setFillColor(sf::Color::White);

    
}
    

// Draw robot's pose means and ground truth
void Visualization::drawPoseAndGroundtruth(const std::vector<Pose>& poseMeans,
                                           const std::vector<double>& sampled_x,
                                           const std::vector<double>& sampled_y,
                                           const std::vector<double>& sampled_theta,
                                           const std::vector<Landmark_Groundtruth> &Landmarks,
                                           int currentStep) {
    // Clear the window
    window_.clear(sf::Color::White);

    // Draw ground truth (sampled positions)
    if (currentStep < sampled_x.size()) {
        //sf::CircleShape groundtruthCircle(20.f); // Increase circle size
        //groundtruthCircle.setFillColor(sf::Color::Green);
        carSprite1.setPosition(scaleFactorX_ * sampled_x[currentStep], -scaleFactorY_ * sampled_y[currentStep]);
        carSprite1.setRotation(poseMeans[currentStep].theta); // Convert radians to degrees
        window_.draw(carSprite1);
    }

    // Draw poseMeans
    if (currentStep < poseMeans.size()) {
        //sf::CircleShape carSprite(20.f); // Increase circle size
        //poseCircle.setFillColor(sf::Color::Blue);
        carSprite.setPosition(scaleFactorX_ * poseMeans[currentStep].x, -scaleFactorY_ * poseMeans[currentStep].y);
        carSprite.setRotation(poseMeans[currentStep].theta); // Convert radians to degrees
        window_.draw(carSprite);

        // Draw orientation line (similar to MATLAB script)
        float lineLength = 50.f; // Adjust line length as needed
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(scaleFactorX_ * poseMeans[currentStep].x, -scaleFactorY_ * poseMeans[currentStep].y)),
            sf::Vertex(sf::Vector2f(scaleFactorX_ * (poseMeans[currentStep].x + lineLength * std::cos(poseMeans[currentStep].theta)),
                                    -scaleFactorY_ * (poseMeans[currentStep].y + lineLength * std::sin(poseMeans[currentStep].theta))))
        };
        line[0].color = sf::Color::Red;
        line[1].color = sf::Color::Red;
        window_.draw(line, 2, sf::Lines);
    }

    // Draw landmarks (assuming they are provided similarly to poseMeans)
    // Adjust according to your data structure
    //std::vector<Pose> landmarks;  // Replace with actual landmark data if available
    for (const auto& landmark : Landmarks) {
        sf::CircleShape landmarkCircle(7.f); // Adjust circle size for landmarks
        landmarkCircle.setFillColor(sf::Color(150, 150, 150)); // Gray color for landmarks
        landmarkCircle.setPosition(scaleFactorX_ * landmark.x, -scaleFactorY_ * landmark.y);
        //landmarkCircle.setRadius(5);
        window_.draw(landmarkCircle);
    }
}

// Display the window
void Visualization::display() {
    window_.display();
}

// Check if the window is open
bool Visualization::isOpen() const {
    return window_.isOpen();
}

// Handle SFML events
void Visualization::handleEvents() {
    sf::Event event;
    while (window_.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window_.close();
        }
    }
}
