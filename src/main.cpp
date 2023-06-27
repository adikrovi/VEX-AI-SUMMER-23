#include "main.h"
#include "EZ-Template/util.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <iostream>


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {1, 2}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{8, 9}

  // IMU Port
  ,6

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1

  // Uncomment if using tracking wheels
  
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  //,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  //,-9 // Rotation sensor
  

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);

pros::Motor left1(11, false);
pros::Motor left2(12, true);
pros::Motor right1(18, true);
pros::Motor right2(19, false);
pros::Motor intake(20, false);
pros::ADIDigitalIn fInput('F');
pros::ADIDigitalIn hInput('H');
pros::ADIDigitalOut eOutput('E');
pros::ADIDigitalOut gOutput('G');
pros::Rotation leftRot(8);
pros::Rotation rightRot(9);
pros::Rotation backRot(10);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

double currX = 0;
double currY = 0;

double prevX = 0;
double prevY = 0;

double pi = M_PI;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

double toRadians(double degrees) {
  return degrees * pi / 180;
}

void odometry() {
  double leftDist = 1.40625;
  double rightDist = 1.40625;
  double backDist = 4.5;

  double wheelDiameter = 2.75;

  double prevLeft = 0;
  double prevRight = 0;
  double prevBack = 0;

  double currLeft;
  double currRight;
  double currBack;

  double deltaLeft;
  double deltaRight;
  double deltaBack;

  double currAngle;
  double prevAngle = 0;
  double deltaAngle;
  double averageAngle;

  double offsetX;
  double offsetY;

  double absoluteX;
  double absoluteY;

  while (true) {
    pros::screen::erase();
    pros::screen::print(TEXT_MEDIUM, 1, "X: %d", (int) currX);
    pros::screen::print(TEXT_MEDIUM, 2, "Y: %d", (int) currY);

    currLeft = leftRot.get_position() / 100.0;
    currRight = -rightRot.get_position() / 100.0;
    currBack = backRot.get_position() / 100.0;

    pros::screen::print(TEXT_MEDIUM, 3, "Orig: L: %d, R: %d, B: %d", (int) currLeft, (int) currRight, (int) currBack);


    deltaLeft = ((currLeft - prevLeft) / 360) * 2 * pi * (wheelDiameter / 2);
    deltaRight = ((currRight - prevRight) / 360) * 2 * pi * (wheelDiameter / 2);
    deltaBack = ((currBack - prevBack) / 360) * 2 * pi * (wheelDiameter / 2);

    pros::screen::print(TEXT_MEDIUM, 4, "Delta: L: %d, R: %d, B: %d", (int) deltaLeft, (int) deltaRight, (int) deltaBack);

    prevLeft = currLeft;
    prevRight = currRight;
    prevBack = currBack;

    currAngle = prevAngle + ((deltaLeft - deltaRight) / (leftDist + rightDist));
    deltaAngle = currAngle - prevAngle;

    if (deltaAngle == 0) {
      offsetX = deltaBack;
      offsetY = deltaRight;
    } else {
      offsetX = 2 * sin(toRadians(currAngle / 2)) * ((deltaBack / deltaAngle) + backDist);
      offsetY = 2 * sin(toRadians(currAngle / 2)) * ((deltaRight / deltaAngle) + rightDist);
    }

    pros::screen::print(TEXT_MEDIUM, 5, "Offset: X: %d, Y: %d", (int) offsetX, (int) offsetY);

    averageAngle = prevAngle + (deltaAngle / 2);

    prevAngle = currAngle;

    
    absoluteX = offsetX * cos(toRadians(averageAngle));
    absoluteY = offsetY * cos(toRadians(averageAngle));
    

    prevX = currX;
    prevY = currY;

    currX += absoluteX;
    currY += absoluteY;
    pros::delay(100);
  }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



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
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}



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
  // int i = 0;
  // int fVal = 0;
  // int hVal = 0;
  // int mult = 128;
  // while (true) {

  //   if (i < 8) {
  //     int fIn = fInput.get_value();
  //     int hIn = hInput.get_value();

  //     switch (i) {
  //       case 0:
  //         fVal += fIn * -1 * mult;
  //         hVal += hIn * -1 * mult;
  //       default:
  //         fVal += fIn * mult;
  //         hVal += hIn * mult;
  //     }

  //     mult /= 2;
  //     i++;
  //   } else {
  //     left1.move(fVal);
  //     left2.move(fVal);
  //     right1.move(hVal);
  //     right2.move(hVal);

  //     i = 0;
  //     fVal = 0;
  //     hVal = 0;
  //     mult = 128;
  //   }

    //pros::delay(100); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  //}

  leftRot.set_position(0);
  rightRot.set_position(0);
  backRot.set_position(0);
  pros::screen::erase();

  pros::Task odom(odometry);

  while (true) {
    int leftPower = controller.get_analog(ANALOG_LEFT_Y) + controller.get_analog(ANALOG_RIGHT_X);
    int rightPower = controller.get_analog(ANALOG_LEFT_Y) - controller.get_analog(ANALOG_RIGHT_X);

    left1.move(leftPower);
    left2.move(leftPower);
    right1.move(rightPower);
    right2.move(rightPower);

    pros::delay(100);
  }

}
