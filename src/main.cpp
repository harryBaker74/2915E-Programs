#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"

void initialize() 
{
	
}

void autonomous() 
{

}

void opcontrol() 
{   
    //Creating Drivetrain
    drivetrain::drivetrain drivetrain = drivetrain::drivetrain();
    subsystems::intake intake = subsystems::intake();
    subsystems::plunger plunger = subsystems::plunger();
    subsystems::mogo mogo = subsystems::mogo();

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