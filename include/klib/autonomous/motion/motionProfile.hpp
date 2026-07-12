#pragma once
#include <vector>
#include "path.hpp"
#include "trajectory.hpp"

namespace klib::motion {

    struct TimeTrajectoryPoint {
        double time = 0;
        double distance = 0;
        double x = 0;
        double y = 0;
        double heading = 0;
        double velocity = 0;
        double angularVelocity = 0;
        double leftVelocity = 0;
        double rightVelocity = 0;
        double leftAcceleration = 0;
        double rightAcceleration = 0;
    };

    class MotionProfile {
        public:
        MotionProfile(
            std::vector<TrajectoryPoint> distanceTrajectory,
            double trackWidth,
            double dt = 0.01);

        TimeTrajectoryPoint sample(double time) const;
        double getTotalTime() const;
        double getDt() const { return dt; }

        const std::vector<TrajectoryPoint> &getDistanceTrajectory() const { return distanceTrajectory; }
        const std::vector<TimeTrajectoryPoint> &getTimeTrajectory() const { return timeTrajectory; }

        private:
        std::vector<TrajectoryPoint> distanceTrajectory;
        std::vector<TimeTrajectoryPoint> timeTrajectory;
        double trackWidth;
        double dt;

        TrajectoryPoint interpolateAtDistance(double distance) const;
        void buildTimeTrajectory();
    };

} // klib::motion
