#pragma once
#include <cmath>
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "klib/utils/pid.hpp"


namespace klib {

    class Drivetrain {
        public:
        Drivetrain(
            pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors,
            pros::IMU &imu,
            PID &lateralPID,
            PID &angularPID,
            float wheelSize,
            float driveRPM,
            float trackWidth,
            pros::Distance &frontDistanceSensor,
            pros::Distance &rightDistanceSensor,
            pros::Distance &backDistanceSensor,
            pros::Distance &leftDistanceSensor
        );

        private:
        pros::MotorGroup &leftMotors;
        pros::MotorGroup &rightMotors;
        pros::IMU &imu;
        PID &lateralPID;
        PID &angularPID;
        pros::Distance frontDistanceSensor;
        pros::Distance rightDistanceSensor;
        pros::Distance backDistanceSensor;
        pros::Distance leftDistanceSensor;

        const float wheelSize;
        const float driveRPM;
        const float trackWidth;
    };

} // klib
