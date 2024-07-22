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


void autonomous() 
{   
    drivetrain.runOdom({0, 0, 0});

    drivetrain.moveToPose(Pose(60, 60, 150), 0.5, 0, false, false, false);
    drivetrain.moveToPose(Pose(0, 0, -90), 0.8, 0, false, false, false);
    drivetrain.turnToHeading(0, false, false);
    drivetrain.moveToPose(Pose(60, 60, 150), 0.5, 0, false, false, false);
    drivetrain.moveToPose(Pose(0, 0, -90), 0.8, 0, false, false, false);
    drivetrain.turnToHeading(0, false, false);
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