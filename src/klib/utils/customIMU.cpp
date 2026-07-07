#include "klib/utils/customIMU.hpp"
#include <cmath>

namespace klib {

    CustomIMU::CustomIMU(pros::Imu &imu, float scaleFactor) :
        imu(imu),
        scaleFactor(scaleFactor)
        {}

    // Radians
    double CustomIMU::getInertialHeading() {
        return imu.get_heading() * M_PI / 180.0 * scaleFactor;
    }

    double CustomIMU::normaliseTarget() {
        double heading = getInertialHeading();
        while (heading < 0) {
            heading += (2 * M_PI);
        }
        while (heading >= 2 * M_PI) {
            heading -= (2 * M_PI);
        }
        return heading;
    }

    double CustomIMU::setIMUHeading(double targetHeading) {
        imu.set_heading(targetHeading / scaleFactor);
        return getInertialHeading();
    }

} // klib
