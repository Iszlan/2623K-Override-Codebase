#include "klib/klib.hpp"
#include "pros/imu.hpp"


namespace klib {

    Drivetrain::Drivetrain(
            pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors,
            CustomIMU &customIMU,
            PID &lateralPID,
            PID &angularPID,
            float wheelSize,
            float driveRPM,
            float trackWidth,
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
        wheelSize(wheelSize),
        driveRPM(driveRPM),
        trackWidth(trackWidth),
        frontDistanceSensor(frontDistanceSensor),
        rightDistanceSensor(rightDistanceSensor),
        backDistanceSensor(backDistanceSensor),
        leftDistanceSensor(leftDistanceSensor)
        {}

} // klib
