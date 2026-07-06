#include "klib/utils/customIMU.hpp"

namespace klib {

    CustomIMU::CustomIMU(pros::Imu &imu, float scaleFactor) :
        imu(imu),
        scaleFactor(scaleFactor)
        {}

    double CustomIMU::getInertialHeading() {
        return imu.get_heading() * scaleFactor;
    }

    double CustomIMU::normaliseTarget() {
        double heading = getInertialHeading();
        while (heading < 0) {
            heading += 360;
        }
        while (heading >= 360) {
            heading -= 360;
        }
        return heading;
    }

    double CustomIMU::setIMUHeading(double targetHeading) {
        imu.set_heading(targetHeading / scaleFactor);
        return getInertialHeading();
    }

} // klib
