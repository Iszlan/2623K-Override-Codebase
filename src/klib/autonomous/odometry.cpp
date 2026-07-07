#include "klib/autonomous/odometry.hpp"
#include <cmath>

namespace klib {

    Odometry::Odometry(
            pros::MotorGroup &leftMotors,
            pros::MotorGroup &rightMotors,
            CustomIMU &customIMU,
            double wheelDiameter,
            double gearRatio,
            double trackWidth) :
        leftMotors(leftMotors),
        rightMotors(rightMotors),
        customIMU(customIMU),
        wheelCircumference(M_PI * wheelDiameter),
        gearRatio(gearRatio),
        trackWidth(trackWidth)
        {}

    void Odometry::startTask() {
        if (task != nullptr) return; // prevents multiple of same task from being created

        // Initialise Positions
        prevLeftPos = leftMotors.get_position();
        prevRightPos = rightMotors.get_position();
        prevHeading = customIMU.getInertialHeading();

        task = new pros::Task([this] {
            while (true) {
                update();
                pros::delay(10);
            }
        });
    }

    void Odometry::stopTask() {
        if (task != nullptr) {
            task -> remove();
            delete task;
            task = nullptr;
        }
    }

    void Odometry::update() {
        
    }

    Pose Odometry::getPose() const {
        poseMutex.take(TIMEOUT_MAX);
        Pose copy = pose;
        poseMutex.give();
        return copy;
    }

    void Odometry::setPose(Pose newPose) {
        poseMutex.take(TIMEOUT_MAX);
        pose = newPose;
        poseMutex.give();
        prevHeading = newPose.theta;
    }

} // klib
