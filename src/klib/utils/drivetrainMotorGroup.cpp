#include "klib/utils/drivetrainMotorGroup.hpp"
#include "pros/motors.hpp"

namespace klib {

    DrivetrainMotorGroup::DrivetrainMotorGroup(pros::Motor &motor1, pros::Motor &motor2, pros::Motor &halfMotor3) :
        motor1(motor1),
        motor2(motor2),
        halfMotor3(halfMotor3)
        {}
    
    void DrivetrainMotorGroup::moveVoltage(int voltage) {
        motor1.move_voltage(voltage);
        motor2.move_voltage(voltage);
        halfMotor3.move_voltage((voltage * 8) / 12); // Scale down the voltage of the 5.5W motor
    }

    double DrivetrainMotorGroup::getAveragePosition() {
        double averagePosition = (motor1.get_position() + motor2.get_position()) / 2.0;
        return averagePosition;
    }

    void DrivetrainMotorGroup::setBrakeMode(pros::motor_brake_mode_e_t mode) {
        motor1.set_brake_mode(mode);
        motor2.set_brake_mode(mode);
        halfMotor3.set_brake_mode(mode);
    }

} // klib
