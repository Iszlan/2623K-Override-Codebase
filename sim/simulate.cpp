#include <cstdio>
#include <iostream>
#include <filesystem>
#include "klib/autonomous/motion/path.hpp"
#include "klib/autonomous/motion/trajectory.hpp"
#include "klib/autonomous/motion/motionProfile.hpp"

using namespace klib::motion;
namespace fs = std::filesystem;

namespace {

    void writePathPointsCsv(const std::string &filepath, const std::vector<PathPoint> &points) {
        FILE *f = std::fopen(filepath.c_str(), "w");
        std::fprintf(f, "distance,x,y,heading,curvature\n");
        for (const auto &p : points) {
            std::fprintf(f, "%.6f,%.6f,%.6f,%.6f,%.6f\n", p.distance, p.x, p.y, p.heading, p.curvature);
        }
        std::fclose(f);
    }

    void writeTrajectoryStagesCsv(const std::string &filepath, const TrajectoryStages &stages) {
        FILE *f = std::fopen(filepath.c_str(), "w");
        std::fprintf(f, "distance,velocityLimitOnly,forwardPassOnly,final,finalAngularVelocity\n");
        for (size_t i = 0; i < stages.final.size(); ++i) {
            std::fprintf(f, "%.6f,%.6f,%.6f,%.6f,%.6f\n",
                stages.final[i].distance,
                stages.velocityLimitOnly[i].velocity,
                stages.forwardPassOnly[i].velocity,
                stages.final[i].velocity,
                stages.final[i].angularVelocity);
        }
        std::fclose(f);
    }

    void writeTimeTrajectoryCsv(const std::string &filepath, const std::vector<TimeTrajectoryPoint> &points) {
        FILE *f = std::fopen(filepath.c_str(), "w");
        std::fprintf(f, "time,distance,x,y,heading,velocity,angularVelocity,leftVelocity,rightVelocity,leftAcceleration,rightAcceleration\n");
        for (const auto &p : points) {
            std::fprintf(f, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                p.time, p.distance, p.x, p.y, p.heading, p.velocity, p.angularVelocity,
                p.leftVelocity, p.rightVelocity, p.leftAcceleration, p.rightAcceleration);
        }
        std::fclose(f);
    }

} // namespace

int main() {
    fs::create_directories("data");

    // Example 1: doc's own worked example (section 3.1)
    {
        Path docPath = Path::fromControlPoints({{12, 36}, {12, 48}, {36, 36}, {36, 48}});
        std::vector<PathPoint> pathPoints = docPath.generatePathPoints(0.1);
        std::cout << "[doc example] path points: " << pathPoints.size()
                  << ", total arc length: " << docPath.getTotalArcLength() << " in\n";

        TrajectoryConstraints constraints;
        constraints.maxVelocity = 40.0;
        constraints.maxAcceleration = 80.0;
        constraints.maxDeceleration = 80.0;
        constraints.trackWidth = 12.0;

        TrajectoryStages stages = generateTrajectoryStages(pathPoints, constraints);
        MotionProfile profile(stages.final, constraints.trackWidth, 0.01);
        std::cout << "[doc example] total time: " << profile.getTotalTime() << " s\n";

        writePathPointsCsv("data/doc_example_path_points.csv", pathPoints);
        writeTrajectoryStagesCsv("data/doc_example_trajectory_stages.csv", stages);
        writeTimeTrajectoryCsv("data/doc_example_time_trajectory.csv", profile.getTimeTrajectory());
    }

    // Example 2: practical multi-waypoint path across a 144x144in field
    {
        Path fieldPath = Path::fromWaypoints({{-48, -48}, {-24, -12}, {24, 0}, {48, 48}});
        std::vector<PathPoint> pathPoints = fieldPath.generatePathPoints(0.1);
        std::cout << "[field example] path points: " << pathPoints.size()
                  << ", total arc length: " << fieldPath.getTotalArcLength() << " in\n";

        TrajectoryConstraints constraints;
        constraints.maxVelocity = 48.0;
        constraints.maxAcceleration = 90.0;
        constraints.trackWidth = 12.0;

        TrajectoryStages stages = generateTrajectoryStages(pathPoints, constraints);
        MotionProfile profile(stages.final, constraints.trackWidth, 0.01);
        std::cout << "[field example] total time: " << profile.getTotalTime() << " s\n";

        writePathPointsCsv("data/field_example_path_points.csv", pathPoints);
        writeTrajectoryStagesCsv("data/field_example_trajectory_stages.csv", stages);
        writeTimeTrajectoryCsv("data/field_example_time_trajectory.csv", profile.getTimeTrajectory());
    }

    // Example 3: same doc path driven backwards, to visualise the
    // forwards/backwards trajectory feature
    {
        Path docPath = Path::fromControlPoints({{12, 36}, {12, 48}, {36, 36}, {36, 48}});
        std::vector<PathPoint> pathPoints = docPath.generatePathPoints(0.1);

        TrajectoryConstraints constraints;
        constraints.maxVelocity = 40.0;
        constraints.maxAcceleration = 80.0;
        constraints.trackWidth = 12.0;
        constraints.forwards = false;

        std::vector<TrajectoryPoint> trajectory = generateTrajectory(pathPoints, constraints);
        MotionProfile profile(trajectory, constraints.trackWidth, 0.01);
        std::cout << "[backward example] total time: " << profile.getTotalTime() << " s\n";

        writeTimeTrajectoryCsv("data/backward_example_time_trajectory.csv", profile.getTimeTrajectory());
    }

    
    // Custom Route
    {
        Path customPath = Path::fromWaypoints({
            {0, 0},
            {10, 4},
            {24, 4},
            {48, 10},
            {24, 24},
            {0, 10},
            {-10, -5},
            {-24, -24}
        });

        std::vector<PathPoint> pathPoints = customPath.generatePathPoints(0.1);

        TrajectoryConstraints constraints;
        constraints.maxVelocity = 48.0;      // in/s
        constraints.maxAcceleration = 90.0;  // in/s^2
        constraints.trackWidth = 10.75;      // in
        constraints.forwards = true;         // set false to test reverse driving

        std::vector<TrajectoryPoint> trajectory = generateTrajectory(pathPoints, constraints);
        MotionProfile profile(trajectory, constraints.trackWidth, 0.01);
        std::cout << "[custom route] path length: " << customPath.getTotalArcLength()
                   << " in, total time: " << profile.getTotalTime() << " s\n";

        writePathPointsCsv("data/custom_route_path_points.csv", pathPoints);
        writeTimeTrajectoryCsv("data/custom_route_time_trajectory.csv", profile.getTimeTrajectory());
    }

    std::cout << "Done. CSVs written to data/\n";
    return 0;
}
