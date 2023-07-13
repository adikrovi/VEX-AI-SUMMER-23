#include "main.h"
#include "EZ-Template/util.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
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
pros::Imu imu(7);

double currX = 0;
double currY = 0;
double currAngle = 0;

double prevX = 0;
double prevY = 0;

double pi = 3.14159265358979323846;

bool odomBool = true;


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

double truncate(double number_val, int n)
{
    double factor = 1;
    double previous = std::trunc(number_val); // remove integer portion
    number_val -= previous;
    for (int i = 0; i < n; i++) {
        number_val *= 10;
        factor *= 10;
    }
    number_val = std::trunc(number_val);
    number_val /= factor;
    number_val += previous; // add back integer portion
    return number_val;
}

void odometry() {
  // distance from middle
  double leftDist = 1.40625;
  double rightDist = 1.40625;
  double backDist = 4.5;

  double wheelDiameter = 2.75;

// distance
  double prevLeft = 0.0;
  double prevRight = 0.0;
  double prevBack = 0.0;
// current distance
  double currLeft;
  double currRight;
  double currBack;
// change in distance
  double deltaLeft;
  double deltaRight;
  double deltaBack;

//angle of circle
  double prevAngle = 0.0;
  double deltaAngle;
  double averageAngle;

  double offsetX;
  double offsetY;

  double absoluteX;
  double absoluteY;

  while (true) {

    if (odomBool) {
      //pros::screen::erase();
      //pros::screen::print(TEXT_MEDIUM, 1, "X: %d Y: %d", (int) currX, (int) currY);
      //pros::screen::print(TEXT_MEDIUM, 2, "Ang: %d", (int) currAngle);
    }
    //pros::screen::erase();
    //pros::screen::print(TEXT_MEDIUM, 1, "X: %d Y: %d", (int) currX, (int) currY);
    //pros::screen::print(TEXT_MEDIUM, 2, "Ang: %d", (int) currAngle);

    currLeft = toRadians(leftRot.get_position() / 100.0);
    currRight = toRadians(-rightRot.get_position() / 100.0);
    currBack = (toRadians(backRot.get_position() / 100.0));

    //pros::screen::print(TEXT_MEDIUM, 3, "Orig: L: %f, R: %f, B: %f", currLeft, currRight, currBack);

    // figuring out the distance the robot has moved (in)
    deltaLeft = ((currLeft - prevLeft)) * (wheelDiameter / 2.0);
    deltaRight = ((currRight - prevRight)) * (wheelDiameter / 2.0);
    deltaBack = ((currBack - prevBack)) * (wheelDiameter / 2.0);

    //pros::screen::print(TEXT_MEDIUM, 4, "Delta: L: %f, R: %f, B: %f", deltaLeft, deltaRight, deltaBack);
   
    // making current distances
    prevLeft = currLeft;
    prevRight = currRight;
    prevBack = currBack;

    //Figuring out angle
    // currAngle = (prevAngle + ((deltaRight - deltaLeft) / (leftDist + rightDist)));
    // deltaAngle = (currAngle - prevAngle);
    // if (currAngle > (2.0 * pi)) {
    //   currAngle = currAngle - (2.0 * pi);
    // } else if (currAngle < (-2.0 * pi)) {
    //   currAngle = currAngle + (2.0 * pi);
    // }
    double currAngle1 = toRadians(imu.get_heading());

    if (currAngle1 > pi) {
        currAngle1 -= (2 * pi);
    }

    currAngle = -currAngle1;
    // if (currAngle < (2.0 * pi)) {
    //   currAngle -= (2.0 * pi);
    // } else if ( currAngle > 0) {
    //   currAngle += (2.0 * pi);
    // }
    deltaAngle = (currAngle - prevAngle);
    //pros::screen::print(TEXT_MEDIUM, 9, "angles: prev: %f, Curr: %f", prevAngle, currAngle);

    //updating offset
    if (deltaAngle < 0.0001) {
      offsetX = deltaBack;
      offsetY = deltaRight;
    } else if (deltaAngle > -0.0001) {
      offsetX = deltaBack;
      offsetY = deltaRight;
    } else if  (deltaAngle > 0.0001 or deltaAngle < -0.0001) {
      offsetX = (2.0 * (sin(currAngle / 2.0))) * ((deltaBack / deltaAngle) + backDist); // this 
      offsetY = (2.0 * (sin(currAngle / 2.0))) * ((deltaRight / deltaAngle) + rightDist);
    } else {
        offsetX = 0.0;
        offsetY = 0.0;
    }

    //pros::screen::print(TEXT_MEDIUM, 7, "Offset: X: %f, Y: %f", (offsetX), (offsetY));

    averageAngle = prevAngle + (deltaAngle / 2.0);

    prevAngle = currAngle;
    //pros::screen::print(TEXT_MEDIUM, 8, "current angktghjtrt: X: %f", currAngle);
    
    absoluteX = offsetX * cos(averageAngle) + offsetY * sin(averageAngle);
    absoluteY = offsetY * cos(averageAngle) + offsetX * sin(averageAngle);
    
    
    prevX = currX;
    prevY = currY;

    currX += absoluteX;
    currY += absoluteY;
  


    pros::delay(5);
  }
}

// Figuring out the angle it needs to go to get to location
double findAng(int x, int y) {
  double dX = (x - currX);
  double dY = (y - currY);
  double desAng = 0.0;

  desAng = (atan2(dX, dY));
  return -desAng;
}

// figurining out how long the motors have to run to get to desired location
void moveToPoint(int x, int y) {
  double distance = abs(sqrt(pow((x - currX), 2.0) + pow((y - currY), 2.0)));
  double turnPidIntegral = 0.0;
  double integralLimit = 2.0;
  double turnPidDerivative = 0.0;
  double turnPidLastError = 0.0;
  double turnPidDrive = 0.0;
  double latPidIntegral = 0.0;
  double latPidDerivative = 0.0;
  double latPidLastError = 0.0;
  double latPidDrive = 0.0;

  double turnKp = 50.0;
  double turnKi = 1.8;
  double turnKd = 30.0;

  double latKi = 25.0;
  double latKp = 1.05;
  double latKd = 0.4;
  
  odomBool = false;
  while (distance > 1.0) {
    // double turnSpeed = 10.0;
    // double speed = 30.0;
    distance = abs(sqrt(pow((x - currX), 2.0) + pow((y - currY), 2.0)));
    double desAng = findAng(x, y);
    double dAng = desAng - currAngle;
    // pros::screen::erase();
    // pros::screen::print(TEXT_MEDIUM, 1, "X: %d Y: %d", (int) currX, (int) currY);
    // pros::screen::print(TEXT_MEDIUM, 2, "Ang: %d", truncate(currAngle, 2));
    // pros::screen::print(TEXT_MEDIUM, 3, "Dist: %d", (int) distance);
    // pros::screen::print(TEXT_MEDIUM, 4, "DesAng: %3d, dAng: %3d", truncate(desAng, 2), truncate(dAng, 2));

    pros::lcd::clear();
    pros::lcd::print(0, "X: %3f Y: %3f", currX, currY);
    pros::lcd::print(1, "Ang: %3f", currAngle);
    pros::lcd::print(2, "Dist: %3f", distance);
    pros::lcd::print(3, "DesAng: %3f, dAng: %3f", desAng, dAng);
    // desAng = findAng(x, y);
    // dAng = desAng - currAngle;

      if (abs(dAng) > .0005){
        if (turnKi != 0) {
            if (abs(dAng) < integralLimit){
              turnPidIntegral = turnPidIntegral + dAng;
            } else {
             turnPidIntegral = 0.0;
             }
        }
        turnPidDerivative = dAng - turnPidLastError;
        turnPidLastError = dAng;
        turnPidDrive = (turnKp * dAng) + (turnKi * turnPidIntegral) + (turnKd * turnPidDerivative);
        
       } else {
        turnPidIntegral = 0.0;
        turnPidLastError = 0.0;
        turnPidDerivative = 0.0;
        turnPidDrive = 0.0;
       }

      if (distance > 0.1) {

          if (latKi !=0) {
              if (abs(distance) < integralLimit) {
                latPidIntegral = latPidIntegral + distance;
              } else {
                latPidIntegral = 0.0;
              }
          
          }
          latPidDerivative = distance - latPidLastError;
          latPidLastError = distance;
          latPidDrive = (latKp * distance) + (latKi * latPidIntegral) + (latKd * latPidDerivative);
      } else {
        latPidIntegral = 0.0;
        latPidLastError = 0.0;
        latPidDerivative = 0.0;
        latPidDrive = 0.0;
      }

      double leftPower = -turnPidDrive + latPidDrive;
      double rightPower = turnPidDrive + latPidDrive;
      left1.move(leftPower);
      left2.move(leftPower);
      right1.move(rightPower);
      right2.move(rightPower);
    // }
    

    pros::delay(20);
    distance = abs(sqrt(pow(x - currX, 2) + pow(y - currY, 2)));
  }
  odomBool = true;
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
 *  // if (dAng > (pi / 2.0) && ((3.0 * pi) / 2) < dAng) {
    //   turnSpeed = 30.0;
    //   turnSpeed *= -1;
    //   speed = 0.0;
    // }
    // if (dAng == 0) {
    //   double leftPower = (speed * (distance / initDist)); 
    //   double rightPower = (speed * (distance / initDist)); 
    //   left1.move(leftPower);
    //   left2.move(leftPower);
    //   right1.move(rightPower);
    //   right2.move(rightPower);
    // } else {
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
  // chassis.reset_pid_targets(); // Resets PID targets to 0
  // chassis.reset_gyro(); // Reset gyro position to 0
  // chassis.reset_drive_sensor(); // Reset drive sensors to 0
  // chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  // ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.

  leftRot.set_position(0);
  rightRot.set_position(0);
  backRot.set_position(0);

  pros::Task odom(odometry);

  moveToPoint(-7, 7);
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


  pros::Task odom(odometry);

  while (true) {
    
    int leftPower = controller.get_analog(ANALOG_LEFT_Y) + controller.get_analog(ANALOG_RIGHT_X);
    int rightPower = controller.get_analog(ANALOG_LEFT_Y) - controller.get_analog(ANALOG_RIGHT_X);

    left1.move(leftPower);
    left2.move(leftPower);
    right1.move(rightPower);
    right2.move(rightPower);
    if (controller.get_digital(DIGITAL_R2)) {
     
      intake = -120;
    }
      else if (controller.get_digital(DIGITAL_R1)) {
      intake = 120;
    }
    else {
      intake = 0;
      
    }
    pros::screen::erase();
    pros::screen::print(TEXT_MEDIUM, 5, "X: %d Y: %d", (int) currX, (int) currY);
    pros::screen::print(TEXT_MEDIUM, 6, "Ang: %d", (int) currAngle);

 
    pros::delay(100);
  }

}
