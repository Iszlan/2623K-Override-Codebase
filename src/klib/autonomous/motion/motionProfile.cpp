#include "klib/autonomous/motion/motionProfile.hpp"
#include <algorithm>
#include <cmath>

namespace klib::motion {

    MotionProfile::MotionProfile(
            std::vector<TrajectoryPoint> distanceTrajectory,
            double trackWidth,
            double dt) :
        distanceTrajectory(std::move(distanceTrajectory)),
        trackWidth(trackWidth),
        dt(dt)
        {
        buildTimeTrajectory();
    }

    TrajectoryPoint MotionProfile::interpolateAtDistance(double distance) const {
        if (distanceTrajectory.empty()) return {};

        if (distance <= distanceTrajectory.front().distance) return distanceTrajectory.front();
        if (distance >= distanceTrajectory.back().distance) return distanceTrajectory.back();

        auto it = std::lower_bound(
            distanceTrajectory.begin(), distanceTrajectory.end(), distance,
            [](const TrajectoryPoint &point, double d) { return point.distance < d; });

        const TrajectoryPoint &upper = *it;
        const TrajectoryPoint &lower = *(it - 1);

        double span = upper.distance - lower.distance;
        double frac = (span > 1e-9) ? (distance - lower.distance) / span : 0.0;

        TrajectoryPoint result;
        result.x = lower.x + (upper.x - lower.x) * frac;
        result.y = lower.y + (upper.y - lower.y) * frac;
        result.heading = lower.heading + (upper.heading - lower.heading) * frac;
        result.curvature = lower.curvature + (upper.curvature - lower.curvature) * frac;
        result.distance = distance;
        result.velocity = lower.velocity + (upper.velocity - lower.velocity) * frac;
        result.angularVelocity = lower.angularVelocity + (upper.angularVelocity - lower.angularVelocity) * frac;
        return result;
    }

    void MotionProfile::buildTimeTrajectory() {
        timeTrajectory.clear();
        if (distanceTrajectory.empty()) return;

        double totalDistance = distanceTrajectory.back().distance;
        double currDist = 0.0;
        double currTime = 0.0;

        while (currDist < totalDistance) {
            TrajectoryPoint p = interpolateAtDistance(currDist);

            TimeTrajectoryPoint tp;
            tp.time = currTime;
            tp.distance = currDist;
            tp.x = p.x;
            tp.y = p.y;
            tp.heading = p.heading;
            tp.velocity = p.velocity;
            tp.angularVelocity = p.angularVelocity;
            tp.leftVelocity = p.velocity - p.angularVelocity * trackWidth / 2.0;
            tp.rightVelocity = p.velocity + p.angularVelocity * trackWidth / 2.0;

            timeTrajectory.push_back(tp);

            double advance = std::max(std::fabs(p.velocity), 0.5) * dt;
            currDist += advance;
            currTime += dt;
        }

        TrajectoryPoint endPoint = distanceTrajectory.back();
        TimeTrajectoryPoint endTp;
        endTp.time = currTime;
        endTp.distance = endPoint.distance;
        endTp.x = endPoint.x;
        endTp.y = endPoint.y;
        endTp.heading = endPoint.heading;
        endTp.velocity = endPoint.velocity;
        endTp.angularVelocity = endPoint.angularVelocity;
        endTp.leftVelocity = endPoint.velocity - endPoint.angularVelocity * trackWidth / 2.0;
        endTp.rightVelocity = endPoint.velocity + endPoint.angularVelocity * trackWidth / 2.0;
        timeTrajectory.push_back(endTp);

        for (size_t i = 0; i < timeTrajectory.size(); ++i) {
            if (i == 0) {
                timeTrajectory[i].leftAcceleration = 0.0;
                timeTrajectory[i].rightAcceleration = 0.0;
                continue;
            }
            double dtActual = timeTrajectory[i].time - timeTrajectory[i - 1].time;
            if (dtActual < 1e-6) dtActual = dt;

            timeTrajectory[i].leftAcceleration =
                (timeTrajectory[i].leftVelocity - timeTrajectory[i - 1].leftVelocity) / dtActual;
            timeTrajectory[i].rightAcceleration =
                (timeTrajectory[i].rightVelocity - timeTrajectory[i - 1].rightVelocity) / dtActual;
        }
    }

    TimeTrajectoryPoint MotionProfile::sample(double time) const {
        if (timeTrajectory.empty()) return {};
        if (time <= timeTrajectory.front().time) return timeTrajectory.front();
        if (time >= timeTrajectory.back().time) return timeTrajectory.back();

        auto it = std::lower_bound(
            timeTrajectory.begin(), timeTrajectory.end(), time,
            [](const TimeTrajectoryPoint &point, double t) { return point.time < t; });

        const TimeTrajectoryPoint &upper = *it;
        const TimeTrajectoryPoint &lower = *(it - 1);

        double span = upper.time - lower.time;
        double frac = (span > 1e-9) ? (time - lower.time) / span : 0.0;

        TimeTrajectoryPoint result;
        result.time = time;
        result.distance = lower.distance + (upper.distance - lower.distance) * frac;
        result.x = lower.x + (upper.x - lower.x) * frac;
        result.y = lower.y + (upper.y - lower.y) * frac;
        result.heading = lower.heading + (upper.heading - lower.heading) * frac;
        result.velocity = lower.velocity + (upper.velocity - lower.velocity) * frac;
        result.angularVelocity = lower.angularVelocity + (upper.angularVelocity - lower.angularVelocity) * frac;
        result.leftVelocity = lower.leftVelocity + (upper.leftVelocity - lower.leftVelocity) * frac;
        result.rightVelocity = lower.rightVelocity + (upper.rightVelocity - lower.rightVelocity) * frac;
        result.leftAcceleration = lower.leftAcceleration + (upper.leftAcceleration - lower.leftAcceleration) * frac;
        result.rightAcceleration = lower.rightAcceleration + (upper.rightAcceleration - lower.rightAcceleration) * frac;
        return result;
    }

    double MotionProfile::getTotalTime() const {
        return timeTrajectory.empty() ? 0.0 : timeTrajectory.back().time;
    }

} // klib::motion
