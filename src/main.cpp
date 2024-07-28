#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/velocityController.hpp"

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
    subsystems::intake intake = subsystems::intake(INTAKE_B, INTAKE_T);
    //Creating plunger
    subsystems::basket basket = subsystems::basket(BASKET_L, BASKET_R);
    //Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO);

void initialize() 
{
	pros::IMU imu = pros::IMU(INERTIAL);

    imu.reset();
}

enum side
{
    BLUE = false,
    RED = true
};

bool autonSide = RED;

void autonomous() 
{   
    if(!autonSide)
    {
    drivetrain.runOdom({0, 0, 0});

    drivetrain.drive(-40, false);
    drivetrain.turnToHeading(45, 1000, false, false);
    drivetrain.drive(-50, false);

    mogo.setState(true);
    intake.setVoltage(8000);

    pros::delay(1000);

    drivetrain.turnToHeading(90, 1000, false, false);

    drivetrain.drive(85, false);
    pros::delay(2000);
    intake.setVoltage(-8000);

    drivetrain.turnToHeading(90, 1000, false, false);

    mogo.setState(false);

    drivetrain.drive(-80, false);

    drivetrain.turnToHeading(180, 1000, false, false);

    intake.setVoltage(0);

    drivetrain.drive(40, false);

    return;
    }
    else
    {
    drivetrain.runOdom({0, 0, 0});

    drivetrain.drive(-40, false);
    drivetrain.turnToHeading(-45, 1000, false, false);
    drivetrain.drive(-50, false);

    mogo.setState(true);
    intake.setVoltage(8000);

    pros::delay(1000);

    drivetrain.turnToHeading(-90, 1000, false, false);

    drivetrain.drive(85, false);
    pros::delay(2000);
    intake.setVoltage(-8000);

    drivetrain.turnToHeading(-90, 1000, false, false);

    mogo.setState(false);

    drivetrain.drive(-80, false);

    drivetrain.turnToHeading(-180, 1000, false, false);

    intake.setVoltage(0);

    drivetrain.drive(40, false);
    }
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