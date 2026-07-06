#include "klib/klib.hpp"


namespace klib {

    Odometry::Odometry(Drivetrain &drivetrain) :
        drivetrain(drivetrain)
        {}

    void Odometry::update() {
        float heading = drivetrain.customIMU.getInertialHeading();
    }

} // klib
