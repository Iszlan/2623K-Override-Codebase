#include "klib/autonomous/motion/cubicBezier.hpp"
#include <cmath>

namespace klib::motion {

    CubicBezier::CubicBezier(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3) :
        p0(p0), p1(p1), p2(p2), p3(p3)
        {}

    Vector2 CubicBezier::getPoint(double t) const {
        double u = 1.0 - t;
        double uu = u * u;
        double uuu = uu * u;
        double tt = t * t;
        double ttt = tt * t;

        return p0 * uuu + p1 * (3.0 * uu * t) + p2 * (3.0 * u * tt) + p3 * ttt;
    }

    Vector2 CubicBezier::getDerivative(double t) const {
        double u = 1.0 - t;
        return (p1 - p0) * (3.0 * u * u)
             + (p2 - p1) * (6.0 * u * t)
             + (p3 - p2) * (3.0 * t * t);
    }

    Vector2 CubicBezier::getSecondDerivative(double t) const {
        double u = 1.0 - t;
        return (p2 - p1 * 2.0 + p0) * (6.0 * u)
             + (p3 - p2 * 2.0 + p1) * (6.0 * t);
    }

    double CubicBezier::getCurvature(double t) const {
        Vector2 d1 = getDerivative(t);
        Vector2 d2 = getSecondDerivative(t);

        double numerator = d1.x * d2.y - d2.x * d1.y;
        double speedSq = d1.x * d1.x + d1.y * d1.y;
        double denominator = std::pow(speedSq, 1.5);

        if (denominator < 1e-9) return 0.0;
        return numerator / denominator;
    }

    double CubicBezier::getHeading(double t) const {
        Vector2 d1 = getDerivative(t);
        return std::atan2(d1.y, d1.x);
    }

} // klib::motion
