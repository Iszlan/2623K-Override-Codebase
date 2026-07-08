#include "klib/klib.hpp" // IWYU pragma: keep
#include <cmath>
#include <algorithm>

namespace klib {

    namespace temp {
        double normaliseAngle(double angle) {
            double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
            if (wrapped < 0) {
                wrapped += 2.0 * M_PI;
            }
            return wrapped - M_PI;
        }

    } // temp namespace


    void Drivetrain::moveToPoint(double x, double y, int timeoutMs, MoveToPointParams params) {
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

        motionTask = new pros::Task([this, x, y, timeoutMs, params] {
            moveToPointTask(x, y, timeoutMs, params);
        });
    }

    void Drivetrain::waitUntil(float distance) {
        while (true) {
            motionMutex.take(TIMEOUT_MAX);
            double travelled = distanceTravelled;
            bool running = motionRunning;
            motionMutex.give();

            if (travelled >= distance || !running) {
                break;
            }

            pros::delay(10);
        }
    }

    void Drivetrain::waitUntilDone() {
        while (true) {
            motionMutex.take(TIMEOUT_MAX);
            bool running = motionRunning;
            motionMutex.give();

            if (!running) {
                break;
            }

            pros::delay(10);
        }
    }

    void Drivetrain::moveToPointTask(double x, double y, int timeoutMs, MoveToPointParams params) {
        lateralPID.reset();
        angularPID.reset();

        uint32_t startTime = pros::millis();
        double maxVoltageMv = params.maxVoltage * 1000.0;
        bool exitedEarly = false;

        Pose startPose = odom.getPose();
        double initialDistance = std::hypot(x - startPose.x, y - startPose.y);

        while ((pros::millis() - startTime) < (uint32_t)timeoutMs) {
            Pose pose = odom.getPose();

            double dx = x - pose.x;
            double dy = y - pose.y;
            double distanceRemaining = std::hypot(dx, dy);

            double travelled = initialDistance - distanceRemaining;

            motionMutex.take(TIMEOUT_MAX);
            distanceTravelled = travelled;
            motionMutex.give();

            if (params.earlyExitRange > 0 && distanceRemaining < params.earlyExitRange) {
                exitedEarly = true;
                break;
            }

            if (distanceRemaining < params.settleDistance) {
                break;
            }

            double targetHeading = params.forwards
                ? std::atan2(dx, dy)
                : std::atan2(-dx, -dy);

            double angleError = temp::normaliseAngle(targetHeading - pose.theta);
            double angleErrorDeg = angleError * (180.0 / M_PI); // angularPID is tuned in degrees

            double distanceError = params.forwards ? distanceRemaining : -distanceRemaining;

            double lateralOutput = lateralPID.update(distanceError) * std::cos(angleError);
            double angularOutput = angularPID.update(angleErrorDeg);

            lateralOutput = std::clamp(lateralOutput, -maxVoltageMv, maxVoltageMv);
            angularOutput = std::clamp(angularOutput, -maxVoltageMv, maxVoltageMv);

            double leftVoltage  = lateralOutput + angularOutput;
            double rightVoltage = lateralOutput - angularOutput;

            double greatest = std::max(std::fabs(leftVoltage), std::fabs(rightVoltage));
            if (greatest > maxVoltageMv) {
                leftVoltage  = (leftVoltage  / greatest) * maxVoltageMv;
                rightVoltage = (rightVoltage / greatest) * maxVoltageMv;
            }

            if (params.minVoltage > 0) {
                double minMv = params.minVoltage * 1000.0;
                if (std::fabs(leftVoltage) > 0 && std::fabs(leftVoltage) < minMv)
                    leftVoltage = std::copysign(minMv, leftVoltage);
                if (std::fabs(rightVoltage) > 0 && std::fabs(rightVoltage) < minMv)
                    rightVoltage = std::copysign(minMv, rightVoltage);
            }

            leftMotors.moveVoltage((int)leftVoltage);
            rightMotors.moveVoltage((int)rightVoltage);

            pros::delay(10);
        }

        if (!exitedEarly) {
            leftMotors.moveVoltage(0);
            rightMotors.moveVoltage(0);
        }

        motionMutex.take(TIMEOUT_MAX);
        motionRunning = false;
        motionMutex.give();
    }

} // klib