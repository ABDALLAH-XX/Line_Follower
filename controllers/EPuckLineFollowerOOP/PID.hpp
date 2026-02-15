#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID(double kp, double ki, double kd);
    double calculate(double error);
    void reset();

private:
    double mKp, mKi, mKd;
    double mLastError = 0.0;
    double mIntegral = 0.0;
};

#endif
