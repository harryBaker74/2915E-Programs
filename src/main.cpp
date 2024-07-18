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
    subsystems::plunger plunger = subsystems::plunger(PLUNGER, ARM_PISTON, CLAMP_PISTON);
    //Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO);

void initialize() 
{
	pros::IMU imu = pros::IMU(INERTIAL);

    imu.reset();
}


void autonomous() 
{   
    drivetrain.runOdom({0, 0, 0});

    drivetrain.moveToPoint(Point(80, 80));

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
        plunger.driverFunctions();
        //Controlling Mogo
        mogo.driverFunctions();
        
        pros::delay(10);
    }
}