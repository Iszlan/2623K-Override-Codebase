#pragma once
#include <cmath>
#include "pros/rtos.hpp"
#include "klib/utils/customIMU.hpp"
#include "pros/distance.hpp"
#include "klib/utils/pid.hpp"
#include "klib/utils/drivetrainMotorGroup.hpp"
#include "klib/autonomous/odometry.hpp"
#include "klib/autonomous/motions/moveToPoint.hpp"
#include "klib/autonomous/motions/turnToAngle.hpp"


namespace klib {

    class Drivetrain {
        public:
        Drivetrain(
            DrivetrainMotorGroup &leftMotors,
            DrivetrainMotorGroup &rightMotors,
            CustomIMU &customIMU,
            PID &lateralPID,
            PID &angularPID,
            Odometry &odom,
            pros::Distance &frontDistanceSensor,
            pros::Distance &rightDistanceSensor,
            pros::Distance &backDistanceSensor,
            pros::Distance &leftDistanceSensor
        );

        void moveToPoint(double x, double y, int timeoutMs, MoveToPointParams params = {});
        void waitUntil(float distance);
        void waitUntilDone();

        void turnToAngle(double angle, int timeoutMs, TurnToAngleParams params = {});

        void setBrakeMode(pros::motor_brake_mode_e_t mode);

        private:
        void moveToPointTask(double x, double y, int timeoutMs, MoveToPointParams params);
        void turnToAngleTask(double angle, int timeoutMs, TurnToAngleParams params);

        DrivetrainMotorGroup &leftMotors;
        DrivetrainMotorGroup &rightMotors;
        CustomIMU &customIMU;
        PID &lateralPID;
        PID &angularPID;
        Odometry &odom;
        pros::Distance &frontDistanceSensor;
        pros::Distance &rightDistanceSensor;
        pros::Distance &backDistanceSensor;
        pros::Distance &leftDistanceSensor;

        pros::Task* motionTask = nullptr;
        bool motionRunning = false;
        double distanceTravelled = 0;
        mutable pros::Mutex motionMutex;
    };

} // klib