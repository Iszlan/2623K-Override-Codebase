#include <cstdio>
#include <iostream>
#include <filesystem>
#include <cmath>
#include "klib/autonomous/motion/path.hpp"
#include "klib/autonomous/motion/trajectory.hpp"
#include "klib/autonomous/motion/motionProfile.hpp"
#include "klib/autonomous/motion/ramsete.hpp"

using namespace klib::motion;
namespace fs = std::filesystem;

int main() {
    fs::create_directories("data");

    Path docPath = Path::fromControlPoints({{12, 36}, {12, 48}, {36, 36}, {36, 48}});
    std::vector<PathPoint> pathPoints = docPath.generatePathPoints(0.1);

    TrajectoryConstraints constraints;
    constraints.maxVelocity = 40.0;
    constraints.maxAcceleration = 80.0;
    constraints.maxDeceleration = 80.0;
    constraints.trackWidth = 12.0;

    std::vector<TrajectoryPoint> trajectory = generateTrajectory(pathPoints, constraints);
    double dt = 0.01;
    MotionProfile profile(trajectory, constraints.trackWidth, dt);

    RamseteGains gains;
    gains.b = 2.0;
    gains.zeta = 0.7;

    Pose2D robotPose;
    robotPose.x = trajectory.front().x + 4.0;
    robotPose.y = trajectory.front().y - 3.0;
    robotPose.theta = trajectory.front().heading + (20.0 * M_PI / 180.0);

    double totalTime = profile.getTotalTime();
    double simDuration = totalTime + 1.0;

    FILE *f = std::fopen("data/ramsete_sim.csv", "w");
    std::fprintf(f, "time,robotX,robotY,robotTheta,targetX,targetY,targetTheta,trackingError,correctedV,correctedOmega\n");

    for (double t = 0.0; t <= simDuration; t += dt) {
        double sampleTime = std::min(t, totalTime);
        TimeTrajectoryPoint target = profile.sample(sampleTime);

        Pose2D desiredPose{target.x, target.y, target.heading};
        RamseteOutput corrected = ramsete(robotPose, desiredPose, target.velocity, target.angularVelocity, gains);

        robotPose.x += corrected.linearVelocity * std::cos(robotPose.theta) * dt;
        robotPose.y += corrected.linearVelocity * std::sin(robotPose.theta) * dt;
        robotPose.theta += corrected.angularVelocity * dt;

        double trackingError = std::hypot(target.x - robotPose.x, target.y - robotPose.y);

        std::fprintf(f, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            t, robotPose.x, robotPose.y, robotPose.theta,
            target.x, target.y, target.heading,
            trackingError, corrected.linearVelocity, corrected.angularVelocity);
    }

    std::fclose(f);
    std::cout << "Ramsete closed-loop simulation written to data/ramsete_sim.csv\n";
    return 0;
}
