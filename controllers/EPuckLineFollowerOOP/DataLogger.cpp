#include "DataLogger.hpp"
#include <cmath>

DataLogger::DataLogger(const std::string& filename, int timeStepMs) {
    mDt = timeStepMs / 1000.0; // Convert ms to seconds
    
    mFile.open(filename);
    if (mFile.is_open()) {
        mFile << "Time,Error,LeftSpeed,RightSpeed,baseSpeed,IAE,ISE\n";
    }
}

DataLogger::~DataLogger() {
    if (mFile.is_open()) mFile.close();
}

void DataLogger::log(const RobotState& state, const double baseSpeed) {
    if (!mFile.is_open()) return;

    // Calculate performance metrics for this timestep and accumulate
    mIae += std::abs(state.lineError) * mDt;
    mIse += (state.lineError * state.lineError) * mDt;

    mFile << state.currentTime << ","
          << state.lineError << ","
          << state.leftSpeed << ","
          << state.rightSpeed << ","
          << baseSpeed << ","
          << mIae << ","
          << mIse << "\n";
}
