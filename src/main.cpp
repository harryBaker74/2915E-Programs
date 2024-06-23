#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/globals.h"

void initialize() 
{
	
}

void autonomous() 
{

}

void opcontrol() 
{   
    //Creating Drivetrain
    drivetrain::drivetrain drivetrain = drivetrain::drivetrain
    (   LEFT_MOTOR_FRONT, 
        LEFT_MOTOR_MID, 
        LEFT_MOTOR_BACK, 
        RIGHT_MOTOR_FRONT, 
        RIGHT_MOTOR_MID, 
        RIGHT_MOTOR_BACK);
    //Creating intake
    subsystems::intake intake = subsystems::intake(INTAKE);
    //Creating plunger
    subsystems::plunger plunger = subsystems::plunger(PLUNGER);
    //Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO);

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