#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/globals.h"

void initialize() 
{
	
}
class test
{
    double num = 0;
    public:
    test(){};
    void testFunct()
    {
        num++;
    }
};


void autonomous() 
{

}

void opcontrol() 
{   
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
    subsystems::plunger plunger = subsystems::plunger(PLUNGER, ARM_PISTON, CLAMP_PISTON);
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