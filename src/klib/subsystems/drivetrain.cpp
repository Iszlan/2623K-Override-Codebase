#include "klib/klib.hpp" // IWYU pragma: keep


namespace klib {

    Drivetrain::Drivetrain(
            klib::DrivetrainMotorGroup &leftMotors,
            klib::DrivetrainMotorGroup &rightMotors,
            CustomIMU &customIMU,
            PID &lateralPID,
            PID &angularPID,
            Odometry &odom,
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
        odom(odom),
        frontDistanceSensor(frontDistanceSensor),
        rightDistanceSensor(rightDistanceSensor),
        backDistanceSensor(backDistanceSensor),
        leftDistanceSensor(leftDistanceSensor)
        {}

        void Drivetrain::setBrakeMode(pros::motor_brake_mode_e_t mode) {
            leftMotors.setBrakeMode(mode);
            rightMotors.setBrakeMode(mode);
        }

} // klib
