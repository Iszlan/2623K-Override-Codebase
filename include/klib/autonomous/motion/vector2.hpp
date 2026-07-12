#pragma once
#include <cmath>

namespace klib::motion {

    struct Vector2 {
        double x = 0;
        double y = 0;

        Vector2 operator+(const Vector2 &other) const { return {x + other.x, y + other.y}; }
        Vector2 operator-(const Vector2 &other) const { return {x - other.x, y - other.y}; }
        Vector2 operator*(double scalar) const { return {x * scalar, y * scalar}; }
        Vector2 operator/(double scalar) const { return {x / scalar, y / scalar}; }

        double magnitude() const { return std::hypot(x, y); }
    };

} // klib::motion
