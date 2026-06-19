    #include "main.h"
    #include "lemlib/api.hpp"
    #include "lemlib/chassis/chassis.hpp"
    #include "lemlib/chassis/trackingWheel.hpp"
    #include "liblvgl/llemu.hpp"
    #include "pros/imu.hpp"
    #include "pros/motors.hpp"
    #include "pros/rtos.hpp"    
    #include <cmath>

    // Drivetrain Configuration 
    pros::MotorGroup left_motors({-1, -3, -2}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
    pros::MotorGroup right_motors({9, 10, 8}, pros::MotorGearset::blue); // right motors use 600 RPM cartridges

    // Drivetrain Settings
    lemlib::Drivetrain drivetrain(
		& left_motors,
        & right_motors,
        11,
        lemlib::Omniwheel::NEW_325,
        360,
        1
    );

    // lateral PID controller
    lemlib::ControllerSettings lateral_controller
    (10, // proportional gain (kP)
    0, // integral gain (kI)
    12, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    200, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
    );

    // angular PID controller
    lemlib::ControllerSettings angular_controller(
        1.2,   // kP slightly lower than original 8
        0,
        2.15,  // kD slightly higher than original 40
        0,
        2,          
        100,
        5,
        300,
        10
    );
        
    // imu
    pros::Imu imu(7);

    // vertical tracking wheel encoder
    pros::Rotation vertical_encoder(6);

    lemlib::TrackingWheel vertical_tracking_wheel(
        &vertical_encoder,
        2.125,
        0.55
    );

    // odometry settings
    lemlib::OdomSensors sensors(
        &vertical_tracking_wheel,
        nullptr,
        nullptr,
        nullptr,
        &imu
    );

    // chassis
    lemlib::Chassis chassis(
        drivetrain,
        lateral_controller,
        angular_controller,
        sensors
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

        pros::lcd::print(0, "initializing + calibrating");
        vertical_encoder.reset();
        vertical_encoder.set_reversed(true);
        chassis.calibrate();


    }

    /**
    * Runs while the robot is in the disabled state of Field Management System or
    * the VEX Competition Switch, following either autonomous or opcontrol. aWhen
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

    void autonomous() {
        chassis.setPose(0, 0, 0);
        pros::delay(1000);

        pros::Task* screen_task = new pros::Task([]() {
            while (true) {
                pros::lcd::print(0, "ROT %ld", vertical_encoder.get_position());
                pros::lcd::print(1, "X %.2f", chassis.getPose().x);
                pros::lcd::print(2, "Y %.2f", chassis.getPose().y);
                pros::lcd::print(3, "Theta %.2f", chassis.getPose().theta);
                pros::delay(500);
            }
        });

        // Test 1: drive forward 24 inches
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 24, 15000, {.maxSpeed = 50}); // slower speed
        chassis.turnToHeading(270, 8000, {.maxSpeed = 50});
        chassis.moveToPoint(-24, 24, 15000, {.maxSpeed = 50}); // slower speed
        chassis.turnToHeading(0, 8000, {.maxSpeed = 50});
        chassis.moveToPoint(-24, 48, 15000, {.maxSpeed = 50}); // slower speed
        chassis.turnToHeading(90, 8000, {.maxSpeed = 50});
        chassis.moveToPoint(0, 48, 15000, {.maxSpeed = 50}); // slower speed
        chassis.turnToHeading(180, 8000, {.maxSpeed = 50});
        chassis.moveToPoint(0, 24, 15000, {.maxSpeed = 50}); // slower speed
        chassis.turnToHeading(0, 8000, {.maxSpeed = 50});
        chassis.moveToPoint(0, 0, 15000, {.forwards = false, .maxSpeed = 50}); // slower speed
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
    void opcontrol() {}