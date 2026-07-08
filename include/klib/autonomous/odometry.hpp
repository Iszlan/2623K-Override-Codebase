#pragma once
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "klib/utils/pose.hpp"
#include "klib/utils/customIMU.hpp"
#include "klib/utils/drivetrainMotorGroup.hpp"

namespace klib {

    class Odometry {
        public:
        Odometry(
            DrivetrainMotorGroup &leftMotors,
            DrivetrainMotorGroup &rightMotors,
            CustomIMU &customIMU,
            double wheelDiameter,
            double gearRatio,
            double trackWidth
        );

        void startTask();
        void stopTask();

        Pose getPose() const;
        void setPose(Pose newPose);

        private:
        void update();

        DrivetrainMotorGroup &leftMotors;
        DrivetrainMotorGroup &rightMotors;
        CustomIMU &customIMU;

        const double wheelCircumference;
        const double gearRatio;
        const double trackWidth;

        double prevLeftPos = 0;
        double prevRightPos = 0;
        double prevHeading = 0; // radians

        Pose pose{};
        mutable pros::Mutex poseMutex;

        pros::Task* task = nullptr;
    };

} // namespace klib