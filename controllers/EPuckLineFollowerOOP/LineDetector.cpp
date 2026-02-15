#include "LineDetector.hpp"

LineDetector::LineDetector(int width, int height) : mWidth(width), mHeight(height) {
    mSlice = cv::Rect(0, height * 0.8, width, height * 0.2);
}

void LineDetector::process(const unsigned char* rawImage, RobotState& state) {
    cv::Mat img(mHeight, mWidth, CV_8UC4, (void*)rawImage);
    cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
    cv::threshold(img, img, 60, 255, cv::THRESH_BINARY_INV);

    cv::Moments m = cv::moments(img(mSlice), true);
    
    if (m.m00 > 0) {
        state.lineFound = true;
        state.lineError = (m.m10 / m.m00) - (mWidth / 2.0);
    } else {
        state.lineFound = false;
    }
    
    // Debug : convert to BGR for visualization
    cv::cvtColor(img, state.debugFrame, cv::COLOR_GRAY2BGR);
}
