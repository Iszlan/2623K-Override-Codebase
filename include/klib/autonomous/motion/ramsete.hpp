#pragma once

namespace klib::motion {

    struct Pose2D {
        double x = 0;
        double y = 0;
        double theta = 0;
    };

    struct RamseteGains {
        double b = 2.0;
        double zeta = 0.7;
    };

    struct RamseteOutput {
        double linearVelocity = 0;
        double angularVelocity = 0;
    };

    RamseteOutput ramsete(
        const Pose2D &currentPose,
        const Pose2D &desiredPose,
        double desiredLinearVelocity,
        double desiredAngularVelocity,
        const RamseteGains &gains = {}
    );

} // klib::motion
