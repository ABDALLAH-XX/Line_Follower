#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include "RobotState.hpp"
#include "LineDetector.hpp"
#include "PID.hpp"
#include "DataLogger.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>



using namespace webots;

int main(int argc, char **argv) {
    // 1. Robot initialization and timestep definition
    Robot *robot = new Robot();
    int timeStep = 32;
    const double MAX_SPEED = 6.28;

    // 2. Hardware initialization
    Camera *cam = robot->getCamera("camera");
    cam->enable(timeStep);

    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    PositionSensor *leftEncoder = robot->getPositionSensor("left wheel sensor");
    PositionSensor *rightEncoder = robot->getPositionSensor("right wheel sensor");
    leftEncoder->enable(timeStep);
    rightEncoder->enable(timeStep);
  
    robot->step(timeStep);

    double initialLeftPos = leftEncoder->getValue();
    double initialRightPos = rightEncoder->getValue();

    double startTime = robot->getTime();

    // 3. Object initialization
    RobotState state;
    LineDetector detector(cam->getWidth(), cam->getHeight());
    PID steeringPID(0.16, 0.0001, 0.001); // To tune
    DataLogger logger("../../analysis/pidtest2_performance.csv", timeStep);

    // 4. Control parameters
    const double baseSpeed = 5.8;
    const double wheelRadius = 0.0205;
    const double targetDistance = 12.4;
    
    

    // --- MAIN LOOP ---
    while (robot->step(timeStep) != -1) {
        // A. UPDATE PERCEPTION 
        state.currentTime = robot->getTime() - startTime;
        state.distanceTraveled = wheelRadius * ((leftEncoder->getValue() - initialLeftPos) + (rightEncoder->getValue() - initialRightPos)) / 2.0;
        detector.process(cam->getImage(), state);

        // B. DECIDE COMMAND
         
        if (state.lineFound) {
            double correction = steeringPID.calculate(state.lineError);
            state.leftSpeed = baseSpeed + correction;
            state.rightSpeed = baseSpeed - correction;
        } 
        else {
            // If line is lost, we can either stop or try to find it by turning in place
            state.leftSpeed = 1.0;
            state.rightSpeed = -1.0;
            //steeringPID.reset();
        }

        // C. STOP CONDITION
        if (state.distanceTraveled > targetDistance) {
            std::cout << "Distance covered:" << state.distanceTraveled << "m." << std::endl;
            std::cout << "Time:" << state.currentTime << "s" << std::endl;
            state.leftSpeed = 0.0;
            state.rightSpeed = 0.0;
        }

        // D. ACTION - We clamp the speeds to the max allowed by Webots
        state.leftSpeed = std::max(-MAX_SPEED, std::min(MAX_SPEED, state.leftSpeed));
        state.rightSpeed = std::max(-MAX_SPEED, std::min(MAX_SPEED, state.rightSpeed));
        leftMotor->setVelocity(state.leftSpeed);
        rightMotor->setVelocity(state.rightSpeed);

        // E. LOGGING & DEBUG
        logger.log(state, baseSpeed);

        if (state.distanceTraveled > targetDistance) 
            break;
        
        // Optional : Show debug image with line detection
        if (!state.debugFrame.empty()) {
            cv::imshow("Robot View", state.debugFrame);
            cv::waitKey(1);
        }

        
    }

    // 5. Cleanup
    delete robot;
    cv::destroyAllWindows();
    return 0;
}
