#include "klib/utils/normaliseAngle.hpp"
#include <cmath>

namespace radians {
    double normaliseAngle(double angle) {
        double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (wrapped < 0) {
            wrapped += 2.0 * M_PI;
        }
        return wrapped - M_PI;
    }

} // radians