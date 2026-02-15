#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include <opencv2/core.hpp>

struct RobotState {
    // Perception
    double lineError = 0.0;
    bool lineFound = false;
    double distanceTraveled = 0.0; 

    // Command
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    bool shouldStop = false;

    // Debug 
    cv::Mat debugFrame;
    double currentTime = 0.0;     // <-- Et ici
};

#endif
