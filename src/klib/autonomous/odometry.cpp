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
        // Average Pos of Motors (Index 2 is 5.5W 200RPM Motor)
        double leftPos = (leftMotors.get_position(0) + leftMotors.get_position(1) + (leftMotors.get_position(2)*3)) / 3.0;
        double rightPos = (rightMotors.get_position(0) + rightMotors.get_position(1) + (rightMotors.get_position(2)*3)) / 3.0;
        double heading = customIMU.getInertialHeading();

        double deltaLeftDeg = leftPos - prevLeftPos;
        double deltaRightDeg = rightPos - prevRightPos;
        prevLeftPos = leftPos;
        prevRightPos = rightPos;

        // Distance Travelled
        double deltaLeftDist = (deltaLeftDeg / 360.0) * gearRatio * wheelCircumference;
        double deltaRightDist = (deltaRightDeg / 360.0) * gearRatio * wheelCircumference;
        double deltaDist = (deltaLeftDist + deltaRightDist) / 2.0;

        double deltaTheta = heading - prevHeading;

        // Normalise
        while (deltaTheta > M_PI) deltaTheta -= (2 * M_PI);
        while (deltaTheta <= -M_PI) deltaTheta += (2 * M_PI);

        double localX, localY;

        // Prevents divide by 0 error
        if (fabs(deltaTheta) < 1e-6) {
            localX = 0;
            localY = deltaDist;
        } 
        else {
            double radius = deltaDist / deltaTheta;
            localX = radius * sin(deltaTheta);
            localY = radius * (1 - cos(deltaTheta));
        }
        
        double globalDeltaX = localX * cos(prevHeading) + localY * sin(prevHeading);
        double globalDeltaY = -localX * sin(prevHeading) + localY * cos(prevHeading);

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

} // namespace klib
