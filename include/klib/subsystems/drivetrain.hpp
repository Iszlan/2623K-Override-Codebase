#pragma once
#include <cmath>
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "klib/utils/customIMU.hpp"
#include "klib/utils/pid.hpp"
#include "klib/utils/drivetrainMotorGroup.hpp"
#include "klib/autonomous/odometry.hpp"
#include "klib/autonomous/motions/moveToPoint.hpp"
#include "klib/autonomous/motions/turnToAngle.hpp"
#include "klib/autonomous/followMotionProfile.hpp"
#include "klib/autonomous/motion/motionProfile.hpp"


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

        void setBrakeMode(pros::motor_brake_mode_e_t mode);

        void moveToPoint(double x, double y, int timeoutMs, MoveToPointParams params = {});
        void turnToAngle(double angleDegrees, int timeoutMs, TurnToAngleParams params = {});
        void followMotionProfile(const klib::motion::MotionProfile &profile, MotionProfileFollowerParams params = {});

        void waitUntil(float distance);
        void waitUntilDone();

        private:
        void moveToPointTask(double x, double y, int timeoutMs, MoveToPointParams params);
        void turnToAngleTask(double angleDegrees, int timeoutMs, TurnToAngleParams params);
        void followMotionProfileTask(const klib::motion::MotionProfile &profile, MotionProfileFollowerParams params);

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
