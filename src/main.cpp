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

//True for scoring side, false for mogo side
bool side = false;
//True for blue, false for red
bool colour = false;
bool safeAuton = true; //Doesnt do anything

void autonomous() 
{   
    drivetrain.runOdom({0, 0, 0});

    drivetrain.moveToPoint(Point(60, 60), false, false);
    /*
    if(safeAuton)
    {
        drivetrain.drive(20, 12000, false);
    }
    else
    {
        if((side) && (!colour))
        {
            //Scoring Red side
            drivetrain.drive(-25, 12000, false);
            drivetrain.turnToHeading(40, 1000, false, false);
            drivetrain.drive(-34, 6000, false);
            mogo.setState(true);
            drivetrain.stop(300);
            intake.setVoltage(12000);
            drivetrain.turnToHeading(130, 1000, false, false);
            drivetrain.drive(32, 7000, false);
            drivetrain.stop(300);
            drivetrain.drive(-20, 7000, false);
            drivetrain.turnToHeading(115, 1000, false, false);
            drivetrain.drive(30, 7000, false);
            drivetrain.stop(300);
            drivetrain.drive(-48, 7000, false);
            drivetrain.turnToHeading(90, 1000, false, false);
            drivetrain.drive(37, 8000, false);
            drivetrain.stop(400);
            drivetrain.drive(-32, 8000, false);
            drivetrain.turnToHeading(180, 1000, false, false);
            drivetrain.drive(40, 4000, false);
        }
        else if ((side) && (colour))
        {
            //Scoring Blue Side
            drivetrain.drive(-25, 12000, false);
            drivetrain.turnToHeading(-40, 1000, false, false);
            drivetrain.drive(-34, 6000, false);
            mogo.setState(true);
            drivetrain.stop(300);
            intake.setVoltage(12000);
            drivetrain.turnToHeading(-130, 1000, false, false);
            drivetrain.drive(32, 7000, false);
            drivetrain.stop(300);
            drivetrain.drive(-20, 7000, false);
            drivetrain.turnToHeading(-115, 1000, false, false);
            drivetrain.drive(30, 7000, false);
            drivetrain.stop(300);
            drivetrain.drive(-48, 7000, false);
            drivetrain.turnToHeading(-90, 1000, false, false);
            drivetrain.drive(37, 8000, false);
            drivetrain.stop(400);
            drivetrain.drive(-32, 8000, false);
            drivetrain.turnToHeading(-180, 1000, false, false);
            drivetrain.drive(40, 4000, false);
        }
        else if ((!side) && (!colour))
        {
            //Mogo Red Side
            drivetrain.drive(-25, 12000, false);
            drivetrain.turnToHeading(-40, 1000, false, false);
            drivetrain.drive(-34, 6000, false);
            mogo.setState(true);
            drivetrain.stop(300);
            intake.setVoltage(12000);
            drivetrain.drive(5, 7000, false);
            drivetrain.turnToHeading(-90, 1000, false, false);
            drivetrain.drive(30, 8000, false);
            drivetrain.stop(300);
            drivetrain.drive(-35, 8000, false);
            drivetrain.turnToHeading(-180, 1000, false, false);
            drivetrain.drive(40, 4000, false);
        }
        else if ((!side) && (colour))
        {
            //Mogo Blue Side
            drivetrain.drive(-25, 12000, false);
            drivetrain.turnToHeading(40, 1000, false, false);
            drivetrain.drive(-34, 6000, false);
            mogo.setState(true);
            drivetrain.stop(300);
            intake.setVoltage(12000);
            drivetrain.drive(5, 7000, false);
            drivetrain.turnToHeading(90, 1000, false, false);
            drivetrain.drive(30, 8000, false);
            drivetrain.stop(300);
            drivetrain.drive(-35, 8000, false);
            drivetrain.turnToHeading(180, 1000, false, false);
            drivetrain.drive(40, 4000, false);
        }
    }
    */

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