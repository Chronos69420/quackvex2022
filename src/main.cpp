#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include <iostream>
#include <cmath>
using namespace okapi;
using namespace std;
#include <stdlib.h>
#include <chrono>

Controller masterController;
Controller partnerController(ControllerId::partner);
Motor frontarm(-12);
Motor backarm(-11);
MotorGroup leftside({-3,-4});
MotorGroup rightside({1,2});
MotorGroup drive({-3, -4, 1, 2});
double xerror;
double yerror;
double heading1;
pros::Gps GPS(20, -1.5, -1.14, 270);
pros::ADIDigitalOut frontpiston ('B');
pros::ADIDigitalOut backpiston ('A');
Motor midside(5);
float Tp = 12000;
float Kp = 0.05385;
float Ki = 0.001011;
float Kd = 0.525;
float movement = 0;
float Last_Error = 0;
int start = 0;


std::shared_ptr<ChassisController> driveController =
ChassisControllerBuilder()
.withMotors(leftside, rightside)
.withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
.build();


std::shared_ptr<OdomChassisController> odom =
ChassisControllerBuilder()
    .withMotors(leftside, rightside)
    .withGains(
        {0.00145, 0.00001, 0.0000115}, //distance
        {0.001864, 0, 0.0000161}, //turn
        {0.0, 0.0, 0.0} //angle
    )
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
    .withMaxVoltage(12000)
    .withOdometry(StateMode::CARTESIAN)
    .buildOdometry();

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 auto midPID = AsyncPosControllerBuilder()
   .withMotor(midside)
   .withGains(
       {0.0, 0.0, 0.0}
 )
   .build();


void initialize() {
	rightside.setBrakeMode(AbstractMotor::brakeMode::coast);
	leftside.setBrakeMode(AbstractMotor::brakeMode::coast);
	midside.setBrakeMode(AbstractMotor::brakeMode::brake);
	frontarm.setBrakeMode(AbstractMotor::brakeMode::hold);
	backarm.setBrakeMode(AbstractMotor::brakeMode::hold);
	pros::Gps GPS(20, -0.16, 0.0);
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

 double x(double targetx) {
 					pros::c::gps_status_s_t status;
 					double n = 0;
 					status = GPS.get_status();
 					while(n < 5) {
 					status = GPS.get_status();
 					//odom -> setState({x*1_in, y*1_in, heading*1_deg});
 					n++;
 					pros::delay(10);
 				  }
 					return targetx - 39.37*status.x;
 				}

 double y(double targety) {
 				pros::c::gps_status_s_t status;
 				double n = 0;
 				status = GPS.get_status();
 				while(n < 5) {
 				status = GPS.get_status();;
 				//odom -> setState({x*1_in, y*1_in, heading*1_deg});
 				n++;
 				pros::delay(10);
 			  }
 				return targety - 39.37*status.y;
 			}


 void Move(double inputx, double inputy) {
					pros::c::gps_status_s_t status;
					double heading1 = GPS.get_heading();
					heading1 = heading1 + 90;
				 	odom -> setState({0_in, 0_in, heading1*1_deg});
					odom -> driveToPoint({x(inputx)*1_in, y(inputy)*1_in});
				}

void rMove(double inputx, double inputy) {
					pros::c::gps_status_s_t status;
					double heading1 = GPS.get_heading();
					heading1 = heading1 + 90;
					odom -> setState({0_in, 0_in, heading1*1_deg});
					odom -> driveToPoint({x(inputx)*1_in, y(inputy)*1_in}, true);
          return;
				}

void turn(double inputx, double inputy) {
					pros::c::gps_status_s_t status;
  				double heading1 = GPS.get_heading();
  				heading1 = heading1 + 90;
					odom -> setState({0_in, 0_in, heading1*1_deg});
        	odom -> turnToPoint({x(inputx)*1_in, y(inputy)*1_in});
        }


void path1(){
    midPID -> flipDisable(false);
    odom -> setMoveThreshold(1_in);
    frontpiston.set_value(true);
    backpiston.set_value(true);
    odom ->setState({0_in,0_in, 45_deg});
    odom ->driveToPoint({15_in, 15_in});
    frontpiston.set_value(false);
    pros::delay(1000);
    frontarm.moveVoltage(12000);
    pros::delay(1500);
    frontarm.moveVoltage(0);
    odom ->driveToPoint({0_in, 40_in}, true);
    backpiston.set_value(true);
    backarm.moveVoltage(1200);
    pros::delay(1000);
    backarm.moveVoltage(0);
    rMove(40,0);
    turn(0,-60);
    backpiston.set_value(false);
    Move(35,0);
    turn(60,0);
    Move(40,0);
    frontpiston.set_value(false);
    rMove(0,20);
    frontarm.moveVoltage(-600);
    backarm.moveVoltage(-600);
    pros::delay(1000);
    frontarm.moveVoltage(0);
    backarm.moveVoltage(0);
    rMove(0,5);
    backpiston.set_value(true);
    backarm.moveVoltage(1200);
    pros::delay(1000);
    backarm.moveVoltage(0);
    Move(0,50);
    frontpiston.set_value(true);
    frontarm.moveVoltage(1200);
    pros::delay(1000);
    frontarm.moveVoltage(0);
    Move(40,20);
    frontpiston.set_value(false);
    Move(35,20);
    turn(-60,20);
    rMove(40,20);
    backpiston.set_value(false);
    Move(-60,60);
    Move(-60,30);
    frontpiston.set_value(true);
    frontarm.moveVoltage(1200);
    pros::delay(1000);
    frontarm.moveVoltage(0);
    rMove(40,60);
    backpiston.set_value(true);
    backarm.moveVoltage(1200);
    pros::delay(1000);
    backarm.moveVoltage(0);
    Move(50,25);
    frontpiston.set_value(false);
    Move(25,-40);
    Move(60,-60);
    frontarm.moveVoltage(-600);
    pros::delay(1000);
    frontarm.moveVoltage(0);
    Move(60,-30);
    frontpiston.set_value(true);
    frontarm.moveVoltage(-600);
    pros::delay(1000);
    frontarm.moveVoltage(0);
    Move(0,-30);
    Move(0,50);
    Move(-60,50);
    Move(-60,0);
    frontarm.moveVoltage(-600);
    frontarm.moveVoltage(-600);
    pros::delay(1000);
    frontarm.moveVoltage(0);
    frontarm.moveVoltage(0);
}



void autonomous() {


  midPID -> flipDisable(false);
  odom -> setMoveThreshold(1_in);
  leftside.moveVoltage(6000);
  rightside.moveVoltage(6000);
  pros::delay(500);
  leftside.moveVoltage(0);
  rightside.moveVoltage(0);
  frontpiston.set_value(false);
  frontarm.moveVoltage(12000);
  pros::delay(1500);
  frontarm.moveVoltage(0);
  pros::delay(500);
  turn(100,32);
  leftside.moveVoltage(-6000);
  rightside.moveVoltage(-7000);
  pros::delay(500);
  frontarm.moveVoltage(-1000);
  pros::delay(200);
  leftside.moveVoltage(0);
  rightside.moveVoltage(0);
  frontarm.moveVoltage(0);
  pros::delay(500);
  leftside.moveVoltage(-7000);
  rightside.moveVoltage(-7000);
  pros::delay(10000);
  leftside.moveVoltage(0);
  rightside.moveVoltage(0);


  // midPID -> flipDisable(false);
  // odom -> setMoveThreshold(1_in);
  // leftside.moveVoltage(6000);
  // rightside.moveVoltage(6000);
  // pros::delay(500);
  // leftside.moveVoltage(0);
  // rightside.moveVoltage(0);
  // frontpiston.set_value(false);
  // frontarm.moveVoltage(12000);
  // pros::delay(1500);
  // frontarm.moveVoltage(0);
  // leftside.moveVoltage(12000);
  // rightside.moveVoltage(12000);
  // pros::delay(6000);
  // leftside.moveVoltage(0);
  // rightside.moveVoltage(0);
  // frontarm.moveVoltage(-12000);
  // pros::delay(3000);
  // frontarm.moveVoltage(0);



  // rMove(11,-36.5);
  //
  //
  // pros::delay(100);
  // backpiston.set_value(false);
  // backarm.moveVoltage(12000);
  // pros::delay(1500);
  // backarm.moveVoltage(0);
  //
  //
  // pros::delay(2000);
  //
  //
  //
  // //
  // // pros::delay(1000);
  // // backpiston.set_value(false);
  // // backarm.moveVoltage(12000);
  // // pros::delay(1500);
  // // backarm.moveVoltage(0);
  //
  //
  //
  // rMove(-40,-10);
  // backpiston.set_value(true);
  // turn(0,-10);
  // frontpiston.set_value(true);
  // backarm.moveVoltage(-10000);
  // pros::delay(1000);
  // backarm.moveVoltage(0);
  // turn(-38, -60);
  // frontarm.moveVoltage(-10000);
  // pros::delay(1500);
  // frontarm.moveVoltage(0);
  // Move(-38, -60);
  // frontpiston.set_value(false);
  // frontarm.moveVoltage(12000);
  // pros::delay(1500);
  // frontarm.moveVoltage(0);
  // rMove(-40, -40);
  // rMove(0,0);
  // backpiston.set_value(false);
  // backarm.moveVoltage(12000);
  // pros::delay(1500);
  // backarm.moveVoltage(0);
  // Move(40,0);
  // frontpiston.set_value(true);
  // rMove(-40,0);
  // backpiston.set_value(true);
  // Move(-40,40);
  // frontarm.moveVoltage(-10000);
  // backarm.moveVoltage(-10000);
  // pros::delay(1500);
  // frontarm.moveVoltage(0);
  // backarm.moveVoltage(0);
  // Move(0,40);
  // frontpiston.set_value(false);
  // frontarm.moveVoltage(12000);
  // pros::delay(1500);
  // frontarm.moveVoltage(0);
  // Move(40,40);
  // rMove(40,60);
  // backpiston.set_value(false);
  // backarm.moveVoltage(12000);
  // pros::delay(1500);
  // backarm.moveVoltage(0);
  // Move(40,40);
  // Move(-40,40);
  // Move(-40,20);
  // turn(-60,20);
  // frontpiston.set_value(true);
  // turn(-40,0);
  // turn(0,20);
  // backpiston.set_value(true);
  // turn(-40,0);
  // backarm.moveVoltage(-10000);
  // pros::delay(1500);
  // backarm.moveVoltage(0);
  // rMove(-40,40);
  // rMove(-60,30);
  // backpiston.set_value(false);
  // Move(-40,40);
  // frontarm.moveVoltage(-10000);
  // backarm.moveVoltage(12000);
  // pros::delay(1500);
  // frontarm.moveVoltage(0);
  // backarm.moveVoltage(0);
  // Move(60,40);
  // turn(60,60);
  // frontarm.moveVoltage(-10000);
  // pros::delay(1500);
  // frontarm.moveVoltage(0);
  // odom -> setState({0_in,0_in,0_deg});
  // odom -> driveToPoint({0_in,20_in});
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
	midPID -> flipDisable(true);
  while(true){
    driveController -> getModel() -> tank(masterController.getAnalog(ControllerAnalog::leftY), masterController.getAnalog(ControllerAnalog::rightY));
    frontarm.moveVoltage(partnerController.getAnalog(ControllerAnalog::leftY)*12000);
    backarm.moveVoltage(partnerController.getAnalog(ControllerAnalog::rightY)*12000);
    if (partnerController.getDigital(ControllerDigital::L1)){
      frontpiston.set_value(true);
    } else if (partnerController.getDigital(ControllerDigital::L2)){
      frontpiston.set_value(false);

    }
    if (partnerController.getDigital(ControllerDigital::R1)){
      backpiston.set_value(true);
    } else if (partnerController.getDigital(ControllerDigital::R2)){
      backpiston.set_value(false);
    }

   if (masterController.getDigital(ControllerDigital::L1)){
      midside.moveVoltage(12000);
   } else if (masterController.getDigital(ControllerDigital::R1)){
      midside.moveVoltage(-12000);
   } else {
      midside.moveVoltage(0);
 }
 }
}
