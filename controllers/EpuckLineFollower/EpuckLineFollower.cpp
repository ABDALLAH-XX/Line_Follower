// File:          EpuckLineFollower.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#define TIME_STEP 32
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  // Initialize devices
  
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  PositionSensor *leftEncoder = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightEncoder = robot->getPositionSensor("right wheel sensor");
  leftEncoder->enable(TIME_STEP);
  rightEncoder->enable(TIME_STEP);
  
  robot->step(TIME_STEP);
  
  double wheelRadius = 0.02;
  
  
  Camera *cam = robot->getCamera("camera");
  if (cam) {
    cam->enable(TIME_STEP);
    cv::namedWindow("Vision Robot", cv::WINDOW_NORMAL);
    cv::resizeWindow("Vision Robot", 320, 240);
  }
  
  cv::Mat visuFrame;
  
  int width = cam->getWidth();
  int height = cam->getHeight();
  
  cv::Rect slice(0, height*0.8, width, height*0.2);
  
  double startTime = robot->getTime();
  double initialLeftPos = leftEncoder->getValue();
  double initialRightPos = rightEncoder->getValue();
  double lastDistance = 0.0;
  
  double baseSpeed = 5.8;
  double kp = 0.015;
  double kd = 0.03;
  double ki = 0.005;
  
  double lastError = 0.0;
  double integral = 0.0;

  
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:      

    double currentDist = wheelRadius * ((leftEncoder->getValue() - initialLeftPos) + 
                                    (rightEncoder->getValue() - initialRightPos)) / 2.0;
    
    double speed = (currentDist - lastDistance) / (TIME_STEP / 1000.0);
    lastDistance = currentDist;

    // Get image from webots
    const unsigned char *image = cam->getImage();

    
    // Converting it into cv gray image with opencv
    cv::Mat img(height, width, CV_8UC4, (void *)image);
    cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
    
    
    cv::threshold(img, img, 60, 255, 1); 
    
    // We crop a window in the center of the camera
    
    cv::Mat active_zone = img(slice);
    
    // Find the center of the line of the sliced image using moments
    cv::Moments m = cv::moments(active_zone, true);
    
    // Visualization
    cv::cvtColor(img, visuFrame, cv::COLOR_GRAY2BGR);
    
    if (m.m00 > 0) {
      int lineCenter = m.m10 / m.m00;
      int imageCenter = width / 2;
      
      // PID command is used. we can also use P, PD and PI controllers
      
      int error = lineCenter - imageCenter;
      
     
      double derivative = error - lastError;
      integral += error;
      
      
      if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0) || error == 0) {
            integral = 0;
      }
      
      //double pCorrection = (kp * error);
      //double pdCorrection = (kp * error) + (kd * derivative);
      //double piCorrection = (kp * error) + (ki * derivative);
      double pidCorrection = (kp * error) + (ki * integral) + (kd * derivative);
      
      
      double leftSpeed = baseSpeed + pidCorrection;
      double rightSpeed = baseSpeed - pidCorrection;
      
      if (leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
      if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
      if (leftSpeed < -MAX_SPEED) leftSpeed = -MAX_SPEED;
      if (rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;
      
      leftMotor->setVelocity(leftSpeed);
      rightMotor->setVelocity(rightSpeed);
      
      lastError = error;
      
      // draw the center line and the center detected
      cv::line(visuFrame, 
                   cv::Point(imageCenter, 0),
                   cv::Point(imageCenter, height),
                   cv::Scalar(255, 0, 0),
                   1);    
      cv::drawMarker(visuFrame, 
                   cv::Point(lineCenter, height * 0.9), 
                   cv::Scalar(0, 0, 255),   
                   cv::MARKER_CROSS,             
                   10,                           
                   2);
      if (error> 2.0 || error < -2.0)
        std::cout << "Line shift : " << error << std::endl;
      
   
    }
    else {
      std::cout << "Line lost. Searching..." << std::endl;
      leftMotor->setVelocity(1.0);
      rightMotor->setVelocity(-1.0);
    }

    
    // Chronometer management
    double currentTime = robot->getTime() - startTime;
    
    // Display time on the opencv image
    std::string timeStr = "Time: " + std::to_string(currentTime).substr(0, 4) + "s";
    cv::putText(visuFrame, timeStr, cv::Point(10, 20), 
      cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
      
    std::string distStr = "Dist: " + std::to_string(currentDist).substr(0, 4) + " m";
    cv::putText(visuFrame, distStr, cv::Point(10, 40), 
            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 0), 1);
            
    std::string speedStr = "Speed: " + std::to_string(speed).substr(0, 4) + " m/s";
    cv::putText(visuFrame, speedStr, cv::Point(10, 60), 
            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 255), 1);
            
    cv::rectangle(visuFrame, slice, cv::Scalar(0, 255, 0), 1);
    
    cv::imshow("Vision Robot", visuFrame);
    cv::waitKey(1);

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
