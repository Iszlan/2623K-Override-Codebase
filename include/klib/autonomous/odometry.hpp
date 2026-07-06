#pragma once
#include "klib/subsystems/drivetrain.hpp"


namespace klib {

    class Odometry {
        public:
        Odometry(Drivetrain &drivetrain);

        void update();

        private:
        Drivetrain &drivetrain;

        double x = 0;
        double y = 0;
        double heading = 0;
    };

} // klib
