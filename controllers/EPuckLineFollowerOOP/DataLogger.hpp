#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include "RobotState.hpp"
#include <fstream>
#include <string>

class DataLogger {
public:
    DataLogger(const std::string& filename, int timeStepMs);
    ~DataLogger();
    
    void log(const RobotState& state, const double baseSpeed);

private:
    std::ofstream mFile;
    double mDt;
    double mIae = 0.0;
    double mIse = 0.0;
};

#endif
