#pragma once
#include <cmath>
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "klib/utils/customIMU.hpp"
#include "pros/distance.hpp"
#include "klib/utils/pid.hpp"
#include "klib/utils/drivetrainMotorGroup.hpp"


namespace klib {

    class Drivetrain {
        public:
        Drivetrain(
            DrivetrainMotorGroup &leftMotors,
            DrivetrainMotorGroup &rightMotors,
            CustomIMU &customIMU,
            PID &lateralPID,
            PID &angularPID,
            pros::Distance &frontDistanceSensor,
            pros::Distance &rightDistanceSensor,
            pros::Distance &backDistanceSensor,
            pros::Distance &leftDistanceSensor
        );

        private:
        DrivetrainMotorGroup &leftMotors;
        DrivetrainMotorGroup &rightMotors;
        CustomIMU &customIMU;
        PID &lateralPID;
        PID &angularPID;
        pros::Distance &frontDistanceSensor;
        pros::Distance &rightDistanceSensor;
        pros::Distance &backDistanceSensor;
        pros::Distance &leftDistanceSensor;
    };

} // klib
