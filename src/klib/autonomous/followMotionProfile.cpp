#include "klib/klib.hpp"
#include "klib/autonomous/motion/motionProfile.hpp"
#include "klib/autonomous/motion/ramsete.hpp"
#include "klib/utils/normaliseAngle.hpp"
#include <cmath>
#include <algorithm>

namespace klib {

    namespace {

        double convertHeadingConvention(double heading) {
            return radians::normaliseAngle((M_PI / 2.0) - heading);
        }

        double signOf(double value) {
            if (value > 1e-6) return 1.0;
            if (value < -1e-6) return -1.0;
            return 0.0;
        }

    } // namespace


    void Drivetrain::followMotionProfile(const klib::motion::MotionProfile &profile, MotionProfileFollowerParams params) {
        waitUntilDone();

        if (motionTask != nullptr) {
            motionTask->remove();
            delete motionTask;
            motionTask = nullptr;
        }

        motionMutex.take(TIMEOUT_MAX);
        distanceTravelled = 0;
        motionRunning = true;
        motionMutex.give();

        motionTask = new pros::Task([this, profile, params] {
            followMotionProfileTask(profile, params);
        });
    }


    void Drivetrain::followMotionProfileTask(const klib::motion::MotionProfile &profile, MotionProfileFollowerParams params) {
        angularPID.reset();

        uint32_t startTime = pros::millis();
        double maxVoltageMv = params.maxVoltage * 1000.0;
        uint32_t totalTimeMs = (uint32_t)(profile.getTotalTime() * 1000.0);

        while ((pros::millis() - startTime) < totalTimeMs) {
            double elapsedSeconds = (pros::millis() - startTime) / 1000.0;
            klib::motion::TimeTrajectoryPoint target = profile.sample(elapsedSeconds);

            motionMutex.take(TIMEOUT_MAX);
            distanceTravelled = target.distance;
            motionMutex.give();

            double leftVoltageMv = 0.0;
            double rightVoltageMv = 0.0;

            if (params.feedbackMode == FeedbackMode::RAMSETE) {
                Pose pose = odom.getPose();
                klib::motion::Pose2D currentPose2D{pose.x, pose.y, convertHeadingConvention(pose.theta)};
                klib::motion::Pose2D desiredPose2D{target.x, target.y, target.heading};

                klib::motion::RamseteOutput corrected = klib::motion::ramsete(
                    currentPose2D, desiredPose2D, target.velocity, target.angularVelocity, params.ramseteGains);

                double correctedLeftVel  = corrected.linearVelocity - corrected.angularVelocity * params.trackWidth / 2.0;
                double correctedRightVel = corrected.linearVelocity + corrected.angularVelocity * params.trackWidth / 2.0;

                double leftFFVolts  = params.kS * signOf(correctedLeftVel)  + params.kV * correctedLeftVel  + params.kA * target.leftAcceleration;
                double rightFFVolts = params.kS * signOf(correctedRightVel) + params.kV * correctedRightVel + params.kA * target.rightAcceleration;

                leftVoltageMv  = leftFFVolts * 1000.0;
                rightVoltageMv = rightFFVolts * 1000.0;

            } else {
                double leftFFVolts  = params.kS * signOf(target.leftVelocity)  + params.kV * target.leftVelocity  + params.kA * target.leftAcceleration;
                double rightFFVolts = params.kS * signOf(target.rightVelocity) + params.kV * target.rightVelocity + params.kA * target.rightAcceleration;

                leftVoltageMv  = leftFFVolts * 1000.0;
                rightVoltageMv = rightFFVolts * 1000.0;

                if (params.feedbackMode == FeedbackMode::HEADING_PID) {
                    Pose pose = odom.getPose();
                    double targetRobotHeading = convertHeadingConvention(target.heading);
                    double headingErrorRad = radians::normaliseAngle(targetRobotHeading - pose.theta);
                    double headingErrorDeg = headingErrorRad * (180.0 / M_PI);

                    double correctionMv = angularPID.update(headingErrorDeg);

                    leftVoltageMv  += correctionMv;
                    rightVoltageMv -= correctionMv;
                }
            }

            leftVoltageMv  = std::clamp(leftVoltageMv, -maxVoltageMv, maxVoltageMv);
            rightVoltageMv = std::clamp(rightVoltageMv, -maxVoltageMv, maxVoltageMv);

            leftMotors.moveVoltage((int)leftVoltageMv);
            rightMotors.moveVoltage((int)rightVoltageMv);

            pros::delay(10);
        }

        leftMotors.moveVoltage(0);
        rightMotors.moveVoltage(0);

        motionMutex.take(TIMEOUT_MAX);
        motionRunning = false;
        motionMutex.give();
    }

} // klib
