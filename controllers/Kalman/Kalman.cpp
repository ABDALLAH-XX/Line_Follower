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
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>


#define TIME_STEP 16
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
  
  GPS *gps = robot->getGPS("gps");
  gps->enable(TIME_STEP);
  
  Compass *compass = robot->getCompass("compass");
  compass->enable(TIME_STEP);
  
  robot->step(TIME_STEP);
  
  
  
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
  
  double wheelRadius = 0.0205;

  
  double startTime = robot->getTime();
  
  double lastTheta = 0.0;
  
  double lastLeftPos = leftEncoder->getValue();
  double lastRightPos = rightEncoder->getValue();
  const double *gpsInit = gps->getValues();
  const double *comValuesInit = compass->getValues();
  lastTheta = atan2(comValuesInit[1], comValuesInit[0]);
  
  double x_start = gpsInit[0];
  double y_start = gpsInit[1];
  
  double x_estimation = x_start;
  double y_estimation = y_start;
  double x_estimation_rk2 = x_start;
  double y_estimation_rk2 = y_start;
  double x_pure = x_start;
  double y_pure = y_start;
  double alpha = 0.98;
  
  double totalDistance = 0.0;
  
  double baseSpeed = 5.8;
  double kp = 0.16;
  double kd = 0.001;
  double ki = 0.0001;
  
  double lastError = 0.0;
  double integral = 0.0;
  
  double iae = 0.0;
  double ise = 0.0;
  
  // save performance to evaluate corrections
  
  std::ofstream logFile("robot_data.csv"); 
  logFile << "Time,StepDist,GPS_X,GPS_Y,Odo_X,Odo_Y,Est_X,Est_Y,rk2_Est_X,rk2_Est_Y,Theta\n";
  
  
  

  
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:      
    
    double currentTime = robot->getTime() - startTime;
    
    double leftPos = leftEncoder->getValue();
    double rightPos = rightEncoder->getValue();
    
    double deltaL = leftPos - lastLeftPos;
    double deltaR = rightPos - lastRightPos;
    const double *gpsPos = gps->getValues();
    const double *comValues = compass->getValues();
    
    
    double theta = atan2(comValues[1], comValues[0]);
    
    // Calcul du déplacement moyen (Point Milieu)
    // On utilise l'angle moyen pour projeter le déplacement
    double avgTheta = lastTheta + (theta - lastTheta) / 2.0;

    double stepDist = wheelRadius * (deltaL + deltaR) / 2.0;
                                    
    double deltaX_rk2 = stepDist * cos(avgTheta);
    double deltaY_rk2 = stepDist * sin(avgTheta);
    
    // Projection et Filtre Complémentaire
    double deltaX = stepDist * cos(theta);
    double deltaY = stepDist * sin(theta);
    
    x_pure += stepDist * cos(theta);
    y_pure += stepDist * sin(theta);
    
    // 4. Mise à jour de l'Estimation Filtrée (Verte)
    // On applique le filtre complémentaire sur la prédiction RK2
    x_estimation_rk2 = alpha * (x_estimation + deltaX_rk2) + (1.0 - alpha) * gpsPos[0];
    y_estimation_rk2 = alpha * (y_estimation + deltaY_rk2) + (1.0 - alpha) * gpsPos[1];
    
    x_estimation = alpha * (x_estimation + deltaX) + (1.0 - alpha) * gpsPos[0];
    y_estimation = alpha * (y_estimation + deltaY) + (1.0 - alpha) * gpsPos[1];
    
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
    lastTheta = theta;
    
    std::cout << "stepDist: " << stepDist << " | Total Dist: " << totalDistance << std::endl;
    
    logFile << robot->getTime() << "," << stepDist << ","
                << gpsPos[0]<< "," << gpsPos[1] << ","
                << x_pure << "," << y_pure << "," 
                << x_estimation << "," << y_estimation << "," 
                << x_estimation_rk2 << "," << y_estimation_rk2 << ","
                << theta << "\n";
    
    double dt = (double)TIME_STEP / 1000.0;
    double speed = stepDist / dt;
    
    totalDistance += stepDist;
    

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
    
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    double error = 0.0;
    
    if (m.m00 > 0) {
      int lineCenter = m.m10 / m.m00;
      int imageCenter = width / 2;
      
      // PID command is used. we can also use P, PD and PI controllers
      
      error = (double)lineCenter - imageCenter;
      
      // adaptafive gain
      // a reduction factor is defined based on normaliszed error
      // the greater error the lower the gain
      double errorNorm = std::abs(error) / (width / 2.0);
      double adaptiveKp = kp * (1.0 - 0.3 * errorNorm); // reduce kp by 50%
      
   
       
      double derivative = error - lastError;
      integral += error;
      
      
      if (error * lastError <= 0) {
            integral = 0;
      }
      
      //double pCorrection = (kp * error);
      //double pdCorrection = (kp * error) + (kd * derivative);
      //double piCorrection = (kp * error) + (ki * derivative);
      double pidCorrection = (adaptiveKp * error) + (ki * integral) + (kd * derivative);
      
      
      leftSpeed = baseSpeed + pidCorrection;
      rightSpeed = baseSpeed - pidCorrection;
      
      double dt = (double) TIME_STEP / 1000.0;
      iae += std::abs(error) * dt;
      ise += error * error * dt;
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
      /*if (error> 2.0 || error < -2.0)
        std::cout << "Line shift : " << error << std::endl;*/
      
   
    }
    else {
      std::cout << "Line lost. Searching..." << std::endl;
      leftSpeed = 1.0;
      rightSpeed = -1.0;
    }
    
    leftSpeed = std::max(-MAX_SPEED, std::min(MAX_SPEED, leftSpeed));
    rightSpeed = std::max(-MAX_SPEED, std::min(MAX_SPEED, rightSpeed));
      
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    
    
    if (totalDistance > 60.0) { 
    std::cout << "Finished ! IAE: " << iae << " | ISE: " << ise << std::endl;
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    break; 
    }
    // Display time on the opencv image
    std::string timeStr = "Time: " + std::to_string(currentTime).substr(0, 4) + "s";
    cv::putText(visuFrame, timeStr, cv::Point(10, 20), 
      cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
      
    std::string distStr = "Dist: " + std::to_string(totalDistance).substr(0, 4) + " m";
    cv::putText(visuFrame, distStr, cv::Point(10, 40), 
            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 0), 1);
            
    std::string speedStr = "Speed: " + std::to_string(speed).substr(0, 4) + " m/s";
    cv::putText(visuFrame, speedStr, cv::Point(10, 60), 
            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 255), 1);
            
    cv::rectangle(visuFrame, slice, cv::Scalar(0, 255, 0), 1);
    
    cv::imshow("Vision Robot", visuFrame);
    cv::waitKey(1);
    

  };
  
  logFile.close();
  cv::destroyAllWindows();
  // Enter here exit cleanup code.
  delete robot;
  return 0;
}
