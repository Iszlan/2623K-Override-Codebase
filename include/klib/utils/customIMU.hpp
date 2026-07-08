#pragma once
#include "pros/imu.hpp"

namespace klib {

    class CustomIMU {
        public:
        CustomIMU(pros::Imu &imu, float scaleFactor);

        double getInertialHeading();
        double setIMUHeading(double targetHeading);

        private:
        pros::Imu &imu;
        float scaleFactor;
    };

} // klib