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
    subsystems::intake intake = subsystems::intake(INTAKE);
    //Creating plunger
    subsystems::basket basket = subsystems::basket(BASKET, BASKET_PISTONS);
    //Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO);

void initialize() 
{
	pros::IMU imu = pros::IMU(INERTIAL);

    imu.reset();


}

//True for blue, because it rhymes
bool side = false;
bool safeAuton = false; //Doesnt do anything

void autonomous() 
{   
    drivetrain.runOdom({0, 0, 0});


    //drivetrain.turnToHeading(-90, 4000, false, false);
    //drivetrain.turnToHeading(-180, 4000, false, false);
    //drivetrain.turnToHeading(-270, 4000, false, false);
    //drivetrain.turnToHeading(-360, 4000, false, false);

    if(safeAuton)
    {
        drivetrain.drive(20, 12000, false);
    }
    else
    {
        if(side)
        {
            //Blue
            drivetrain.drive(-30, 12000, false);
            drivetrain.turnToHeading(45, 1000, false, false);
            drivetrain.drive(-28, 6000, false);
            mogo.setState(true);
            drivetrain.stop(300);
            intake.setVoltage(12000);
            drivetrain.turnToHeading(130, 1000, false, false);
            drivetrain.drive(32, 7000, false);
            drivetrain.stop(400);
            drivetrain.drive(-20, 6000, false);
            drivetrain.turnToHeading(119, 1000, false, false);
            drivetrain.drive(28, 7000, false);
            drivetrain.stop();
            drivetrain.drive(-40, 6000, false);
            drivetrain.stop(400);
            mogo.setState(false);
            drivetrain.turnToHeading(180, 1000, false, false);
            drivetrain.drive(20, 6000, false);
            drivetrain.stop();
        }
        else if (!side)
        {
            //Red
            drivetrain.drive(-30, 12000, false);
            drivetrain.turnToHeading(-45, 1000, false, false);
            drivetrain.drive(-28, 6000, false);
            mogo.setState(true);
            drivetrain.stop(300);
            intake.setVoltage(12000);
            drivetrain.turnToHeading(-130, 1000, false, false);
            drivetrain.drive(32, 7000, false);
            drivetrain.stop(400);
            drivetrain.drive(-20, 6000, false);
            drivetrain.turnToHeading(-119, 1000, false, false);
            drivetrain.drive(28, 7000, false);
            drivetrain.stop();
            drivetrain.drive(-40, 6000, false);
            drivetrain.stop(400);
            mogo.setState(false);
            drivetrain.turnToHeading(-180, 1000, false, false);
            drivetrain.drive(20, 6000, false);
            drivetrain.stop();
        }
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