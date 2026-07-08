#pragma once
#include "pros/motors.hpp"

namespace klib {

    class DrivetrainMotorGroup {
        public:
        DrivetrainMotorGroup(pros::Motor &motor1, pros::Motor &motor2, pros::Motor &halfMotor3);

        void moveVoltage(int voltage); // -12 to 12 then scales 5.5W motor accordingly
        double getAveragePosition();

        void setBrakeMode(pros::motor_brake_mode_e_t mode);

        private:
        pros::Motor &motor1;
        pros::Motor &motor2;
        pros::Motor &halfMotor3;
    };

} // klib
