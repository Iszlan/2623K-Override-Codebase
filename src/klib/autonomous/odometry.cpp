#include "klib/autonomous/odometry.hpp"
#include <cmath>

namespace klib {

    Odometry::Odometry(
            DrivetrainMotorGroup &leftMotors,
            DrivetrainMotorGroup &rightMotors,
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
        prevLeftPos = leftMotors.getAveragePosition();
        prevRightPos = rightMotors.getAveragePosition();
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
        float encoderLagMultiplier = 1.04;

        float leftPos = leftMotors.getAveragePosition();
        float rightPos = rightMotors.getAveragePosition();
        float heading = customIMU.getInertialHeading(); // clockwise

        float deltaLeft = ((leftPos - prevLeftPos) / 360 ) * wheelCircumference * gearRatio;
        float deltaRight = ((rightPos - prevRightPos) / 360 ) * wheelCircumference * gearRatio;
        float deltaDistance = encoderLagMultiplier * ((deltaLeft + deltaRight) / 2.0f);

        float deltaTheta = heading - prevHeading;     
        
        prevLeftPos = leftPos;
        prevRightPos = rightPos;

        float x, y;

        if (std::fabs(deltaTheta) < 1e-6f) {
            y = deltaDistance;
            x = 0.0f;
        } 
        else {
            float radius = deltaDistance / deltaTheta;
            float chordLength = 2.0f * radius * sin(deltaTheta / 2.0f);
            y = chordLength * cos(deltaTheta);
            x = chordLength * sin(deltaTheta);
        }

        float globalDeltaX = -1 * (x * cos(prevHeading) - y * sin(prevHeading));
        float globalDeltaY = x * sin(prevHeading) + y * cos(prevHeading);

        poseMutex.take(TIMEOUT_MAX);
        pose.x += globalDeltaX;
        pose.y += globalDeltaY;
        pose.theta = heading;
        poseMutex.give();

        prevHeading = heading;
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
