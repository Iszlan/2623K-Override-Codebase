#include "main.h"
#include "klib/klib.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

using namespace klib;


// Temporary for Debugging
pros::MotorGroup leftMotors({-13, -12, -11}); // 11 is 5.5
pros::MotorGroup rightMotors({18, 19, 20}); // 20 is 5.5

pros::Imu imu(16);
CustomIMU customIMU(imu, 1.01);

PID lateralPID(10, 0.0, 0.0, true, 0.0, 0.0);
PID angularPID(10, 0.0, 0.0, true, 0.0, 0.0);

pros::Distance frontDistanceSensor(2);
pros::Distance rightDistanceSensor(3);
pros::Distance backDistanceSensor(4);
pros::Distance leftDistanceSensor(5);

Drivetrain drivetrain(
	leftMotors,
	rightMotors,
	customIMU,
	lateralPID,
	angularPID,
	frontDistanceSensor,
	rightDistanceSensor,
	backDistanceSensor,
	leftDistanceSensor
);

Odometry odom(
	leftMotors,
	rightMotors,
	customIMU,
	3.25,
	0.75,
	10.75
);




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	odom.startTask();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		Pose pose = odom.getPose();

        printf("%f,%f,%f\n", pose.x, pose.y, pose.theta);

        pros::delay(10); // 100 Hz
	}
}