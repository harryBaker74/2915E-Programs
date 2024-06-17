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
    drivetrain::drivetrain robotDrivetrain = drivetrain::drivetrain();

	while(true)
    {
        //Controlling Drivetrain
        robotDrivetrain.driverFunctions();
        pros::delay(10);
    }
}