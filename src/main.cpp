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
    cubicBezier curve1 = cubicBezier(Point(10, -85), Point(5, -60), Point(-10, -20), Point(-10, -20));
    profile motionProfile = profile(175.6, 473.4, 275);
    std::vector<std::vector<double>> profile1;

void initialize() 
{
	pros::IMU imu = pros::IMU(INERTIAL);

    imu.reset(false);
    profile1 = motionProfile.generateProfile(curve1, 50, 2.75);

}

enum auton
{
    LEFT = 0,
    RIGHT = 1
};

enum auton Auton = LEFT;

void autonomous() 
{   
    
    drivetrain.runOdom({0, 0, 0});
    intake.setVoltage(12000);
    pros::delay(300);
    intake.setVoltage(0);
    basket.holdPosition(subsystems::LiftPosition::WALLSTAKESCORE);

    if(Auton == LEFT)
    {
    drivetrain.drive(-40, 12000, false);
    drivetrain.turnToHeading(40, 1000, false, false);
    drivetrain.drive(-48, 5000, true);
    drivetrain.waitUntil(30);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.turnToHeading(90, 1000, false, false);
    intake.setVoltage(12000);
    drivetrain.drive(70, 12000, false);
    pros::delay(200);
    drivetrain.drive(-70, 12000, false);
    drivetrain.turnToHeading(-135, 1000, false, false);
    drivetrain.drive(20, 12000, false);
    drivetrain.turnToHeading(-165, 200, false, false);
    drivetrain.turnToHeading(-105, 200, false, false);
    drivetrain.turnToHeading(-165, 200, false, false);
    drivetrain.turnToHeading(-105, 200, false, false);
    drivetrain.turnToHeading(-135, 1000, false, false);
    drivetrain.stop();
    basket.setPosition(subsystems::LiftPosition::DEFAULT);
    pros::delay(400);
    basket.endHold();
    }
    else if (Auton == RIGHT)
    {
    drivetrain.drive(-40, 12000, false);
    drivetrain.turnToHeading(-40, 1000, false, false);
    drivetrain.drive(-48, 5000, true);
    drivetrain.waitUntil(30);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.turnToHeading(-90, 1000, false, false);
    intake.setVoltage(12000);
    drivetrain.drive(70, 12000, false);
    pros::delay(200);
    drivetrain.drive(-70, 12000, false);
    drivetrain.turnToHeading(135, 1000, false, false);
    drivetrain.drive(20, 12000, false);
    drivetrain.turnToHeading(165, 200, false, false);
    drivetrain.turnToHeading(105, 200, false, false);
    drivetrain.turnToHeading(165, 200, false, false);
    drivetrain.turnToHeading(105, 200, false, false);
    drivetrain.turnToHeading(135, 1000, false, false);
    drivetrain.stop();
    basket.setPosition(subsystems::LiftPosition::DEFAULT);
    pros::delay(400);
    basket.endHold();
    }
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