#include "klib/utils/pid.hpp"
#include <cmath>


namespace klib {

    PID::PID(double kp, double ki, double kd, bool resetOnSignFlip, double windupRange, double slewRate) :
        kp(kp),
        ki(ki),
        kd(kd),
        resetOnSignFlip(resetOnSignFlip),
        windupRange(windupRange),
        slewRate(slewRate)
        {}
 
    // Returns 1 if positive, -1 if negative, 0 if exactly zero.
    int PID::signCheck(double value) {
        if (value > 0) return 1;
        if (value < 0) return -1;
        return 0;
    }
 
    void PID::setGains(double newKp, double newKi, double newKd) {
        kp = newKp;
        ki = newKi;
        kd = newKd;
    }
 
    void PID::setWindupRange(double newWindupRange) {
        windupRange = newWindupRange;
    }
 
    void PID::setResetOnSignFlip(bool newResetOnSignFlip) {
        resetOnSignFlip = newResetOnSignFlip;
    }

    void PID::setSlewRate(double newSlewRate) {
        slewRate = newSlewRate;
    }
 
    void PID::reset() {
        integral = 0;
        prevError = 0;
        prevOutput = 0;
        firstUpdate = true;
    }

    double PID::slewLimit(double target) {
        if (slewRate == 0) return target;
 
        double delta = target - prevOutput;
 
        if (fabs(target) > fabs(prevOutput)) {
            if (delta > slewRate) delta = slewRate;
            else if (delta < -slewRate) delta = -slewRate;
        }
 
        return prevOutput + delta;
    }

    double PID::update(double currError) {
        if  (firstUpdate) {
            prevError = currError;
            firstUpdate = false;
        }

        // Setpoint Error
        if (resetOnSignFlip && (signCheck(currError) != signCheck(prevError))) {
            integral = 0;
        }
        // Anti-Windup
        if (windupRange != 0 && fabs(currError) > windupRange) {
            integral = 0;
        } 
        else {
            integral += currError;
        }

        double derivative = currError - prevError;
        prevError = currError;
 
        double rawOutput = (currError * kp) + (integral * ki) + (derivative * kd);
 
        double output = slewLimit(rawOutput);
        prevOutput = output;
 
        return output;
    }

} // klib
