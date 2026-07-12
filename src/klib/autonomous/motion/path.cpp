#include "klib/autonomous/motion/path.hpp"
#include <stdexcept>

namespace klib::motion {

    Path::Path(std::vector<CubicBezier> segments) : segments(std::move(segments)) {}

    Path Path::fromControlPoints(const std::vector<Vector2> &controlPoints) {
        if (controlPoints.size() < 4 || controlPoints.size() % 4 != 0) {
            throw std::invalid_argument("Path::fromControlPoints requires a multiple of 4 points");
        }

        std::vector<CubicBezier> segments;
        segments.reserve(controlPoints.size() / 4);

        for (size_t i = 0; i + 3 < controlPoints.size(); i += 4) {
            segments.emplace_back(
                controlPoints[i], controlPoints[i + 1],
                controlPoints[i + 2], controlPoints[i + 3]);
        }

        return Path(std::move(segments));
    }

    Path Path::fromWaypoints(const std::vector<Vector2> &waypoints) {
        if (waypoints.size() < 2) {
            throw std::invalid_argument("Path::fromWaypoints requires at least 2 waypoints");
        }

        std::vector<CubicBezier> segments;
        segments.reserve(waypoints.size() - 1);

        for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
            size_t prevIdx = (i == 0) ? 0 : i - 1;
            size_t nextIdx = (i + 2 < waypoints.size()) ? i + 2 : waypoints.size() - 1;

            const Vector2 &pPrev = waypoints[prevIdx];
            const Vector2 &pCurr = waypoints[i];
            const Vector2 &pNext = waypoints[i + 1];
            const Vector2 &pNextNext = waypoints[nextIdx];

            Vector2 b0 = pCurr;
            Vector2 b1 = pCurr + (pNext - pPrev) / 6.0;
            Vector2 b2 = pNext - (pNextNext - pCurr) / 6.0;
            Vector2 b3 = pNext;

            segments.emplace_back(b0, b1, b2, b3);
        }

        return Path(std::move(segments));
    }

    std::vector<PathPoint> Path::generatePathPoints(double deltaD) const {
        std::vector<PathPoint> points;
        double cumulativeDistance = 0.0;

        for (size_t segIdx = 0; segIdx < segments.size(); ++segIdx) {
            const CubicBezier &segment = segments[segIdx];

            double t = 0.0;
            while (t < 1.0) {
                Vector2 point = segment.getPoint(t);
                double curvature = segment.getCurvature(t);
                double heading = segment.getHeading(t);

                points.push_back({point.x, point.y, heading, curvature, cumulativeDistance});

                double speed = segment.getDerivative(t).magnitude();
                double deltaT = (speed > 1e-6) ? (deltaD / speed) : 0.01;

                t += deltaT;
                cumulativeDistance += deltaD;
            }

            Vector2 endPoint = segment.getPoint(1.0);
            double endCurvature = segment.getCurvature(1.0);
            double endHeading = segment.getHeading(1.0);

            if (!points.empty()) {
                double dx = endPoint.x - points.back().x;
                double dy = endPoint.y - points.back().y;
                cumulativeDistance = points.back().distance + std::hypot(dx, dy);
            }

            points.push_back({endPoint.x, endPoint.y, endHeading, endCurvature, cumulativeDistance});
        }

        totalArcLength = points.empty() ? 0.0 : points.back().distance;
        return points;
    }

} // klib::motion
