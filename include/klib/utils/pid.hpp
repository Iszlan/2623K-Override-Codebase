#pragma once
#include <cmath>


namespace klib {

    class PID {
        public:
        PID(double kp, double ki, double kd, bool resetOnSignFlip, double windupRange, double slewRate);

        void setGains(double newKp, double newKi, double newKd);
        void setWindupRange(double newWindupRange);
        void setResetOnSignFlip(bool newResetOnSignFlip);
        void setSlewRate(double newSlewRate);

        void reset();

        double update(double error);

        private:
        static int signCheck(double value);
        double slewLimit(double target);

        double kp, ki, kd;
        bool resetOnSignFlip;
        double windupRange;
        double slewRate = 0;

        double integral = 0;
        double prevError = 0;
        double prevOutput = 0;
        bool firstUpdate = true;
    };

} // klib
