#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep	
#include "lemlib/chassis/chassis.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

// Drivetrain Configuration 
pros::MotorGroup left_motors({-8, -10}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({2, 20}, pros::MotorGearset::blue); // right motors use 600 RPM cartridges

// Drivetrain Settings
lemlib::Drivetrain drivetrain(& left_motors, // left motor group
	& right_motors, // right motor group
	11.75, // 12 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
	360, // drivetrain rpm is 360
	8 // horizontal drift is 2 (for now)
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              150, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              50, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              150, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// Intake Configuration 
pros::Motor intake(1); // intake motor on port 1

//Outtake Configuration
pros::Motor hoodroller(14); // hood roller motor on port 14
pros::Motor switchroller(12); // switch roller motor on port 12

//Pneumatics Configuration
pros::adi::DigitalOut matchloader('A'); // Matchloader
pros::adi::DigitalOut wing('H'); // Descore Wing


// imu
pros::Imu imu(7);
// horizontal tracking wheel encoder
// pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
// pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(on_center_button);
    chassis.calibrate();
    while (imu.is_calibrating()) { // waits exactly as long as needed
        pros::delay(10);
    }
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(20);
        }
    });
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
void competition_initialize();

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 32.2, 3000, {.maxSpeed = 120});
    chassis.turnToHeading(90, 3000, {.maxSpeed = 50});
    matchloader.set_value(false);
    intake.move(127);
    pros::delay(500);   
    chassis.moveToPoint(16.5, 32.2, 2500, {.maxSpeed = 100, .minSpeed = 120});
    pros::delay(1150);   
    chassis.moveToPoint(12.5, 32.2, 3000, {.forwards = false, .maxSpeed = 120}); // drive backwards 2.5"
    chassis.turnToHeading(90, 3000, {.maxSpeed = 50});                           // correct heading to 90°
    chassis.moveToPoint(-20, 32.5, 3250, {.forwards = false, .maxSpeed = 120});  // drive backwards 27"
    pros::delay(1250); // run for 1 second
    switchroller.move(-127);                                                      // spin switch roller reverse
    hoodroller.move(-127);   
    pros::delay(1000); // run for 1 second
    switchroller.move(0); // ✅ stop
    hoodroller.move(0);   // ✅ stop                                                     // spin hood roller reverse
} 

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or   the VEX Competition Switch in the operator
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
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        left_motors.move(dir - turn);  // correct motors
        right_motors.move(dir + turn);
        pros::delay(20);
    }
}