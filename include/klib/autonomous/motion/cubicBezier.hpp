#pragma once
#include "vector2.hpp"

namespace klib::motion {

    class CubicBezier {
        public:
        CubicBezier(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3);

        Vector2 getPoint(double t) const;
        Vector2 getDerivative(double t) const;
        Vector2 getSecondDerivative(double t) const;
        double getCurvature(double t) const;
        double getHeading(double t) const;

        const Vector2 p0, p1, p2, p3;
    };

} // klib::motion
