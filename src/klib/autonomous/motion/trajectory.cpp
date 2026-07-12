#include "klib/autonomous/motion/trajectory.hpp"
#include <cmath>
#include <algorithm>

namespace klib::motion {

    double curvatureLimitedVelocity(double curvature, double maxVelocity, double trackWidth) {
        double absCurvature = std::fabs(curvature);
        return (2.0 * maxVelocity) / (2.0 + trackWidth * absCurvature);
    }

    namespace {

        double accelerationLimitedVelocity(double previousVelocity, double accel, double deltaD) {
            double vSquared = previousVelocity * previousVelocity + 2.0 * accel * deltaD;
            return std::sqrt(std::max(0.0, vSquared));
        }

        std::vector<TrajectoryPoint> baseTrajectoryPoints(const std::vector<PathPoint> &pathPoints) {
            std::vector<TrajectoryPoint> trajectory;
            trajectory.reserve(pathPoints.size());
            for (const auto &p : pathPoints) {
                trajectory.push_back({p.x, p.y, p.heading, p.curvature, p.distance, 0.0, 0.0});
            }
            return trajectory;
        }

        void fillAngularVelocities(std::vector<TrajectoryPoint> &trajectory) {
            for (auto &point : trajectory) {
                point.angularVelocity = point.velocity * point.curvature;
            }
        }

        double normaliseAngleLocal(double angle) {
            double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
            if (wrapped < 0) wrapped += 2.0 * M_PI;
            return wrapped - M_PI;
        }

        void applyDirection(std::vector<TrajectoryPoint> &trajectory, bool forwards) {
            if (forwards) return;
            for (auto &point : trajectory) {
                point.velocity = -point.velocity;
                point.heading = normaliseAngleLocal(point.heading + M_PI);
            }
        }

    } // namespace

    TrajectoryStages generateTrajectoryStages(
        const std::vector<PathPoint> &pathPoints,
        const TrajectoryConstraints &constraints) {

        TrajectoryStages stages;
        if (pathPoints.empty()) return stages;

        stages.velocityLimitOnly = baseTrajectoryPoints(pathPoints);
        for (auto &point : stages.velocityLimitOnly) {
            double vCurv = curvatureLimitedVelocity(point.curvature, constraints.maxVelocity, constraints.trackWidth);
            point.velocity = std::min(vCurv, constraints.maxVelocity);
        }
        fillAngularVelocities(stages.velocityLimitOnly);
        applyDirection(stages.velocityLimitOnly, constraints.forwards);

        stages.forwardPassOnly = baseTrajectoryPoints(pathPoints);
        {
            auto &fwd = stages.forwardPassOnly;
            fwd[0].velocity = constraints.startVelocity;
            for (size_t i = 1; i < fwd.size(); ++i) {
                double deltaD = fwd[i].distance - fwd[i - 1].distance;
                double vAccel = accelerationLimitedVelocity(fwd[i - 1].velocity, constraints.maxAcceleration, deltaD);
                double vCurv = curvatureLimitedVelocity(fwd[i].curvature, constraints.maxVelocity, constraints.trackWidth);
                fwd[i].velocity = std::min({vAccel, vCurv, constraints.maxVelocity});
            }
        }
        fillAngularVelocities(stages.forwardPassOnly);

        std::vector<TrajectoryPoint> backward = baseTrajectoryPoints(pathPoints);
        {
            size_t n = backward.size();
            backward[n - 1].velocity = constraints.endVelocity;
            for (size_t i = n - 1; i-- > 0; ) {
                double deltaD = backward[i + 1].distance - backward[i].distance;
                double vAccel = accelerationLimitedVelocity(backward[i + 1].velocity, constraints.maxDeceleration, deltaD);
                double vCurv = curvatureLimitedVelocity(backward[i].curvature, constraints.maxVelocity, constraints.trackWidth);
                backward[i].velocity = std::min({vAccel, vCurv, constraints.maxVelocity});
            }
        }

        stages.final = baseTrajectoryPoints(pathPoints);
        for (size_t i = 0; i < stages.final.size(); ++i) {
            stages.final[i].velocity = std::min(stages.forwardPassOnly[i].velocity, backward[i].velocity);
        }
        fillAngularVelocities(stages.final);

        applyDirection(stages.forwardPassOnly, constraints.forwards);
        applyDirection(stages.final, constraints.forwards);

        return stages;
    }

    std::vector<TrajectoryPoint> generateTrajectory(
        const std::vector<PathPoint> &pathPoints,
        const TrajectoryConstraints &constraints) {
        return generateTrajectoryStages(pathPoints, constraints).final;
    }

} // klib::motion
