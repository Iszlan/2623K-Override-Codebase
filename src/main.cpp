#include "main.h"
#include "klib/klib.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

using namespace klib;


// Temporary for Debugging
pros::Controller con(pros::E_CONTROLLER_MASTER);

pros::Motor leftBack(-13, pros::v5::MotorGears::blue);
pros::Motor leftMid(-12, pros::v5::MotorGears::blue);
pros::Motor leftFront(-11, pros::v5::MotorGears::green);

pros::Motor rightBack(18, pros::v5::MotorGears::blue);
pros::Motor rightMid(19, pros::v5::MotorGears::blue);
pros::Motor rightFront(20, pros::v5::MotorGears::green);

klib::DrivetrainMotorGroup leftMotors(leftBack, leftMid, leftFront);
klib::DrivetrainMotorGroup rightMotors(rightBack, rightMid, rightFront);

pros::Imu imu(16);
CustomIMU customIMU(imu, 1.01);

PID lateralPID(10, 0.0, 0.0, true, 0.0, 0.0);
PID angularPID(10, 0.0, 0.0, true, 0.0, 0.0);

pros::Distance frontDistanceSensor(2);
pros::Distance rightDistanceSensor(3);
pros::Distance backDistanceSensor(4);
pros::Distance leftDistanceSensor(5);

Odometry odom(
	leftMotors,
	rightMotors,
	customIMU,
	3.25,
	0.75,
	10.75
);

Drivetrain drivetrain(
	leftMotors,
	rightMotors,
	customIMU,
	lateralPID,
	angularPID,
	odom,
	frontDistanceSensor,
	rightDistanceSensor,
	backDistanceSensor,
	leftDistanceSensor
);



void initialize() {
	pros::lcd::initialize();

	imu.reset(true);

	odom.startTask();
	pros::delay(100);

	pros::Task([&] {
        while (true) {
            pros::lcd::print(0, "x: %.2f", odom.getPose().x);
			pros::lcd::print(1, "y: %.2f", odom.getPose().y);
			pros::lcd::print(2, "theta: %.2f", odom.getPose().theta * 180.0 / M_PI);
        }
    });
}


void disabled() {}


void competition_initialize() {}


void autonomous() {
	drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	drivetrain.moveToPoint(0, 24, 2000);
}


void opcontrol() {
	drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        pros::delay(10); // 100 Hz
    }
}