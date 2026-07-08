#include "klib/klib.hpp" // IWYU pragma: keep

namespace klib {

    void Drivetrain::waitUntil(float distance) {
        while (true) {
            motionMutex.take(TIMEOUT_MAX);
            double travelled = distanceTravelled;
            bool running = motionRunning;
            motionMutex.give();

            if (travelled >= distance || !running) {
                break;
            }

            pros::delay(10);
        }
    }

    void Drivetrain::waitUntilDone() {
        while (true) {
            motionMutex.take(TIMEOUT_MAX);
            bool running = motionRunning;
            motionMutex.give();

            if (!running) {
                break;
            }

            pros::delay(10);
        }
    }

} // klib
