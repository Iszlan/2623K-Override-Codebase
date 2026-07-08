#include "klib/utils/customIMU.hpp"
#include <cmath>

namespace klib {

    CustomIMU::CustomIMU(pros::Imu &imu, float scaleFactor) :
        imu(imu),
        scaleFactor(scaleFactor)
        {}


    double normaliseAngle(double angle) {
        angle = std::fmod(angle, 360.0);
        if (angle < 0) {
            angle += 360.0;
        }
        return angle * M_PI / 180.0; // Convert to radians
    }

    // Radians
    double CustomIMU::getInertialHeading() {
        return normaliseAngle(imu.get_heading() * scaleFactor);
    }

    double CustomIMU::setIMUHeading(double targetHeading) {
        imu.set_heading(targetHeading / scaleFactor);
        return getInertialHeading();
    }

} // klib
