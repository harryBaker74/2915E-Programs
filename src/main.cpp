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
    
    drivetrain.boomerang(Pose(-60, 120, -90), 6000, 0.5, false, false, false);
    drivetrain.stop(300);

}

void opcontrol() 
{
    drivetrain.stopOdom();
    basket.endHold();
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