#ifndef LINE_DETECTOR_HPP
#define LINE_DETECTOR_HPP

#include "RobotState.hpp"
#include <opencv2/opencv.hpp>

class LineDetector {
public:
    LineDetector(int width, int height);
    void process(const unsigned char* rawImage, RobotState& state);

private:
    int mWidth, mHeight;
    cv::Rect mSlice;
};

#endif
