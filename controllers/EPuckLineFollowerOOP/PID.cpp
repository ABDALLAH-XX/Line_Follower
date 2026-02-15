#include "PID.hpp"
#include <algorithm>

/**
 * Constructor : Initialize coefficients and accumulators
 */
PID::PID(double kp, double ki, double kd) 
    : mKp(kp), mKi(ki), mKd(kd), mLastError(0.0), mIntegral(0.0) {
}

/**
 * Computes signal command
 * @param error : current error (line shift)
 * #@param dt : time elapsed since last operation (in seconds)
 */
double PID::calculate(double error) {
    // 1. Proportional 
    double pTerm = mKp * error;

    // 2. Integral 
    // We multiply by dt make Ki frequency independant
    if (error * mLastError < 0) {
        // If error sign changed, reset integral to avoid overshoot
        mIntegral = 0.0;
    }

    mIntegral += error;
    double iTerm = mKi * mIntegral;

    // 3. Derivative
    double derivative = error - mLastError;
    double dTerm = mKd * derivative;

    // Error update
    mLastError = error;

    
    return pTerm + iTerm + dTerm;
}

/**
 * Reset : Reset every counter.
 * Call it when the obstacle is lost.
 */
void PID::reset() {
    mLastError = 0.0;
    mIntegral = 0.0;
}
