#include "klib/klib.hpp"


namespace klib {

    Drivetrain::Drivetrain(
            pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors,
            CustomIMU &customIMU,
            PID &lateralPID,
            PID &angularPID,
            pros::Distance &frontDistanceSensor,
            pros::Distance &rightDistanceSensor,
            pros::Distance &backDistanceSensor,
            pros::Distance &leftDistanceSensor
        ) :
        leftMotors(leftMotors),
        rightMotors(rightMotors),
        customIMU(customIMU),
        lateralPID(lateralPID),
        angularPID(angularPID),
        frontDistanceSensor(frontDistanceSensor),
        rightDistanceSensor(rightDistanceSensor),
        backDistanceSensor(backDistanceSensor),
        leftDistanceSensor(leftDistanceSensor)
        {}

} // klib
