#include "klib/klib.hpp" // IWYU pragma: keep
#include <cmath>
#include <algorithm>

namespace klib {

    void Drivetrain::turnToAngle(double angleDegrees, int timeoutMs, TurnToAngleParams params) {
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

        motionTask = new pros::Task([this, angleDegrees, timeoutMs, params] {
            turnToAngleTask(angleDegrees, timeoutMs, params);
        });
    }


    void Drivetrain::turnToAngleTask(double angleDegrees, int timeoutMs, TurnToAngleParams params) {
        angularPID.reset();

        uint32_t startTime = pros::millis();

        double maxVoltageMv = params.maxVoltage * 1000.0;
        double minVoltageMv = params.minVoltage * 1000.0;

        double targetAngle = angleDegrees * (M_PI / 180.0);

        double startHeadingDeg = odom.getPose().theta * (180.0 / M_PI);

        int settleTimer = 0;
        bool exitedEarly = false;

        while ((pros::millis() - startTime) < (uint32_t)timeoutMs) {

            Pose pose = odom.getPose();

            double errorRad = radians::normaliseAngle(targetAngle - pose.theta);
            double errorDeg = errorRad * (180.0 / M_PI);

            double currentHeadingDeg = pose.theta * (180.0 / M_PI);
            double headingDeltaDeg = radians::normaliseAngle(
                (currentHeadingDeg - startHeadingDeg) * (M_PI / 180.0)
            ) * (180.0 / M_PI);
            double degreesTurned = std::fabs(headingDeltaDeg);

            motionMutex.take(TIMEOUT_MAX);
            distanceTravelled = degreesTurned;
            motionMutex.give();

            if (params.earlyExitRange > 0 && std::fabs(errorDeg) < params.earlyExitRange) {
                exitedEarly = true;
                break;
            }

            if (std::fabs(errorDeg) < params.settleError) {
                settleTimer += 10;
            } else {
                settleTimer = 0;
            }

            if (settleTimer >= params.settleTime) {
                break;
            }

            double output = angularPID.update(errorDeg);

            // Clamp to max voltage
            output = std::clamp(output, -maxVoltageMv, maxVoltageMv);

            if (params.minVoltage > 0 &&
                std::fabs(output) > 0 &&
                std::fabs(output) < minVoltageMv) {

                output = std::copysign(minVoltageMv, output);
            }

            leftMotors.moveVoltage((int)output);
            rightMotors.moveVoltage((int)-output);


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
