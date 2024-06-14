#include "Visualization.h"

Visualization::Visualization() : window_(sf::VideoMode(800, 600), "Robot Localization Visualization") {
    // Constructor initializes SFML window with size 800x600 and title "Robot Localization Visualization"
    // Set up a view to better visualize the entire scene
    view_.setCenter(400, 300); // Center the view at (400, 300)
    view_.setSize(800, 600);   // Set the view size to match the window size
    window_.setView(view_);
}

void Visualization::drawPoseAndGroundtruth(const std::vector<Pose>& poseMeans,
                                           const std::vector<double>& sampled_x,
                                           const std::vector<double>& sampled_y,
                                           const std::vector<double>& sampled_theta,
                                           int currentStep) {
    // Clear the window
    window_.clear(sf::Color::White);

    // Draw all ground truth positions up to the current step
    for (int i = 0; i <= currentStep && i < sampled_x.size(); ++i) {
        sf::CircleShape groundtruthCircle(2.f);
        groundtruthCircle.setFillColor(sf::Color::Green);
        groundtruthCircle.setPosition(sampled_x[i], sampled_y[i]);
        window_.draw(groundtruthCircle);
    }

    // Draw all pose means up to the current step
    for (int i = 0; i <= currentStep && i < poseMeans.size(); ++i) {
        sf::CircleShape poseCircle(2.f);
        poseCircle.setFillColor(sf::Color::Blue);
        poseCircle.setPosition(poseMeans[i].x, poseMeans[i].y);
        window_.draw(poseCircle);
    }
}

void Visualization::display() {
    // Display the updated window
    window_.display();
}

bool Visualization::isOpen() const {
    // Check if the window is open
    return window_.isOpen();
}

void Visualization::handleEvents() {
    // Handle events (e.g., window close events)
    sf::Event event;
    while (window_.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window_.close();
        }
    }
}
