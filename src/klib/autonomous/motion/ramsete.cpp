#include "klib/autonomous/motion/ramsete.hpp"
#include <cmath>

namespace klib::motion {

    namespace {

        double sinc(double x) {
            if (std::fabs(x) < 1e-6) {
                return 1.0 - (x * x) / 6.0;
            }
            return std::sin(x) / x;
        }

        double normaliseAngleLocal(double angle) {
            double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
            if (wrapped < 0) wrapped += 2.0 * M_PI;
            return wrapped - M_PI;
        }

    } // namespace

    RamseteOutput ramsete(
            const Pose2D &currentPose,
            const Pose2D &desiredPose,
            double desiredLinearVelocity,
            double desiredAngularVelocity,
            const RamseteGains &gains) {

        double dx = desiredPose.x - currentPose.x;
        double dy = desiredPose.y - currentPose.y;

        double cosTheta = std::cos(currentPose.theta);
        double sinTheta = std::sin(currentPose.theta);

        double eX = cosTheta * dx + sinTheta * dy;
        double eY = -sinTheta * dx + cosTheta * dy;
        double eTheta = normaliseAngleLocal(desiredPose.theta - currentPose.theta);

        double k = 2.0 * gains.zeta * std::sqrt(
            desiredAngularVelocity * desiredAngularVelocity
            + gains.b * desiredLinearVelocity * desiredLinearVelocity);

        RamseteOutput output;
        output.linearVelocity = desiredLinearVelocity * std::cos(eTheta) + k * eX;
        output.angularVelocity = desiredAngularVelocity + k * eTheta
            + gains.b * desiredLinearVelocity * sinc(eTheta) * eY;

        return output;
    }

} // klib::motion
