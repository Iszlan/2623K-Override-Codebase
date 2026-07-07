#pragma once
#include <cmath>
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "klib/utils/customIMU.hpp"
#include "pros/distance.hpp"
#include "klib/utils/pid.hpp"


namespace klib {

    class Drivetrain {
        public:
        Drivetrain(
            pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors,
            CustomIMU &customIMU,
            PID &lateralPID,
            PID &angularPID,
            pros::Distance &frontDistanceSensor,
            pros::Distance &rightDistanceSensor,
            pros::Distance &backDistanceSensor,
            pros::Distance &leftDistanceSensor
        );

        private:
        pros::MotorGroup &leftMotors;
        pros::MotorGroup &rightMotors;
        CustomIMU &customIMU;
        PID &lateralPID;
        PID &angularPID;
        pros::Distance &frontDistanceSensor;
        pros::Distance &rightDistanceSensor;
        pros::Distance &backDistanceSensor;
        pros::Distance &leftDistanceSensor;
    };

} // klib
