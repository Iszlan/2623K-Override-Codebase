#pragma once
#include <vector>
#include "path.hpp"

namespace klib::motion {

    struct TrajectoryConstraints {
        double maxVelocity = 40.0;
        double maxAcceleration = 80.0;
        double maxDeceleration = 80.0;
        double trackWidth = 12.0;
        double startVelocity = 0.0;
        double endVelocity = 0.0;
        bool forwards = true;
    };

    struct TrajectoryPoint {
        double x = 0;
        double y = 0;
        double heading = 0;
        double curvature = 0;
        double distance = 0;
        double velocity = 0;
        double angularVelocity = 0;
    };

    struct TrajectoryStages {
        std::vector<TrajectoryPoint> velocityLimitOnly;
        std::vector<TrajectoryPoint> forwardPassOnly;
        std::vector<TrajectoryPoint> final;
    };

    double curvatureLimitedVelocity(double curvature, double maxVelocity, double trackWidth);

    TrajectoryStages generateTrajectoryStages(
        const std::vector<PathPoint> &pathPoints,
        const TrajectoryConstraints &constraints);

    std::vector<TrajectoryPoint> generateTrajectory(
        const std::vector<PathPoint> &pathPoints,
        const TrajectoryConstraints &constraints);

} // klib::motion
