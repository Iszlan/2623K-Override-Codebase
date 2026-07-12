#pragma once
#include <vector>
#include "vector2.hpp"
#include "cubicBezier.hpp"

namespace klib::motion {

    struct PathPoint {
        double x = 0;
        double y = 0;
        double heading = 0;
        double curvature = 0;
        double distance = 0;
    };

    class Path {
        public:
        explicit Path(std::vector<CubicBezier> segments);

        static Path fromControlPoints(const std::vector<Vector2> &controlPoints);
        static Path fromWaypoints(const std::vector<Vector2> &waypoints);

        std::vector<PathPoint> generatePathPoints(double deltaD = 0.1) const;
        double getTotalArcLength() const { return totalArcLength; }

        private:
        std::vector<CubicBezier> segments;
        mutable double totalArcLength = 0;
    };

} // klib::motion
