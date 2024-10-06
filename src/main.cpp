#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/velocityController.hpp"
#include "../include/harryLibHeader/pathGen.hpp"

 //Creating Drivetrain
    subsystems::drivetrain drivetrain = subsystems::drivetrain
    (   LEFT_MOTOR_FRONT, 
        LEFT_MOTOR_MID, 
        LEFT_MOTOR_BACK, 
        RIGHT_MOTOR_FRONT, 
        RIGHT_MOTOR_MID, 
        RIGHT_MOTOR_BACK,
        TRACKING_WHEEL,
        INERTIAL);
    //Creating intake
    subsystems::intake intake = subsystems::intake(INTAKE);
    //Creating plunger
    subsystems::basket basket = subsystems::basket(BASKET, BASKET_PISTONS);
    //Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO);


    //Auton paths and motion profiling
    cubicBezier curve1 = cubicBezier(Point(0, 0), Point(-5, 30), Point(-20, 45), Point(-60, 45));
    cubicBezier curve2 = cubicBezier(Point(-65, 45), Point(-30, 55), Point(-10, 40), Point(-10, 0));
    profile motionProfile = profile(175.6, 473.4, 275);
    std::vector<std::vector<double>> profile1;
    std::vector<std::vector<double>> profile2;

void initialize() 
{
	pros::IMU imu = pros::IMU(INERTIAL);

    imu.reset(false);
    profile1 = motionProfile.generateProfile(curve1, 50, 2.75);
    profile2 = motionProfile.generateProfile(curve2, 50, 2.75);

}

//True for scoring side, false for mogo side
bool side = false;
//True for going left
bool direction = true;
bool safeAuton = false; //Doesnt do anything

void autonomous() 
{   
    
    drivetrain.runOdom({0, 0, 0});

    drivetrain.tangentIntersection(curve1, profile1, false, false);
    drivetrain.tangentIntersection(curve2, profile2, true, false);
    drivetrain.stop();

    /*if(safeAuton)
    {
        intake.setVoltage(-12000);
        drivetrain.drive(-20, 12000, false);
        drivetrain.stop();
    }
    else
    {
    if((side == true) && (direction == false))
    {
    intake.setVoltage(-12000);
    drivetrain.drive(-48, 12000, false);
    mogo.setState(true);
    pros::delay(500);
    drivetrain.drive(-20, 12000, false);
    drivetrain.turnToHeading(97, 1000, false, false);
    intake.setVoltage(12000);
    drivetrain.drive(35, 12000, false);
    drivetrain.turnToHeading(68, 1000, false, false);
    drivetrain.drive(20, 12000, false);
    drivetrain.turnToHeading(75, 1000, false, false);
    drivetrain.drive(-35, 12000, false);
    pros::delay(200);
    drivetrain.turnToHeading(25, 800, false, false);
    drivetrain.drive(28, 12000, false);
    drivetrain.drive(-12, 12000, false);
    drivetrain.turnToHeading(-33, 700, false, false);
    drivetrain.drive(50, 12000, false);
    drivetrain.turnToHeading(192, 1000, false, false);
    drivetrain.drive(75, 12000, false);
        drivetrain.stop();


    }
    else if((side == true) && (direction == true))
    {
    intake.setVoltage(-12000);
    drivetrain.drive(-48, 12000, false);
    mogo.setState(true);
    pros::delay(500);
    drivetrain.drive(-20, 12000, false);
    drivetrain.turnToHeading(-97, 1000, false, false);
    intake.setVoltage(12000);
    drivetrain.drive(35, 12000, false);
    drivetrain.turnToHeading(-68, 1000, false, false);
    drivetrain.drive(20, 12000, false);
    drivetrain.turnToHeading(-75, 1000, false, false);
    drivetrain.drive(-35, 12000, false);
    pros::delay(200);
    drivetrain.turnToHeading(-25, 800, false, false);
    drivetrain.drive(28, 12000, false);
    drivetrain.drive(-12, 12000, false);
    drivetrain.turnToHeading(33, 700, false, false);
    drivetrain.drive(50, 12000, false);
    drivetrain.turnToHeading(-192, 1000, false, false);
    drivetrain.drive(75, 12000, false);
        drivetrain.stop();

    }
    else if ((side == false) && (direction == false))
    {
    intake.setVoltage(-12000);
    drivetrain.drive(-48, 12000, false);
    mogo.setState(true);
    pros::delay(500);
    drivetrain.drive(-20, 12000, false);
    pros::delay(1000);
    intake.setVoltage(12000);
    drivetrain.drive(68, 12000, false);
    drivetrain.drive(-50, 12000, false);
    drivetrain.turnToHeading(45, 1000, false, false);
    drivetrain.drive(40, 12000, false);
    drivetrain.drive(-40, 12000, false);
    drivetrain.turnToHeading(185, 1250, false, false);
    drivetrain.drive(30, 6000, false);
        drivetrain.stop();

    }
    else if ((side == false) && (direction == true))
    {
    intake.setVoltage(-12000);
    drivetrain.drive(-48, 12000, false);
    mogo.setState(true);
    pros::delay(500);
    drivetrain.drive(-20, 12000, false);
    pros::delay(1000);
    intake.setVoltage(12000);
    drivetrain.drive(68, 12000, false);
    drivetrain.drive(-50, 12000, false);
    drivetrain.turnToHeading(-45, 1000, false, false);
    drivetrain.drive(40, 12000, false);
    drivetrain.drive(-40, 12000, false);
    drivetrain.turnToHeading(-185, 1250, false, false);
    drivetrain.drive(30, 6000, false);
        drivetrain.stop();

    }
    }*/
}

void opcontrol() 
{ 
    
    
	while(true)
    {

        //Controlling Drivetrain
        drivetrain.driverFunctions();
        //Controlling Intake
        intake.driverFunctions();
        //Controlling Plunger
        basket.driverFunctions();
        //Controlling Mogo
        mogo.driverFunctions();
        
        pros::delay(10);
    }
}