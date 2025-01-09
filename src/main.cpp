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
        INERTIAL
    );
//Creating intake
    subsystems::intake intake = subsystems::intake(INTAKE, OPTICAL);
//Creating Lift
    subsystems::lift lift = subsystems::lift(LIFT_1, LIFT_2);
//Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO);
//Creating Mogo
    subsystems::rushMech rushMech = subsystems::rushMech(RUSH);
//Creating Mogo
    subsystems::doinker doinker = subsystems::doinker(DOINKER);
//Creating Mogo
    subsystems::pto pto = subsystems::pto(PTO);


//Auton paths and motion profiling
profile motionProfile = profile(200, 300, 275);

cubicBezier curve1 = cubicBezier(Point(0, 0), Point(0, 45), Point(15, 60), Point(60, 60));
std::vector<std::vector<double>> profile1 = motionProfile.generateProfile(curve1, 50, 3);

void initialize() 
{
	pros::IMU imu = pros::IMU(INERTIAL);
    imu.reset(false);
}

enum auton
{
    LEFT = 0,
    RIGHT = 1
};

enum auton Auton = LEFT;

void autonomous() 
{   
    drivetrain.runOdom(Pose (0, 0, 0));
    intake.setRingSortColour(true);
    //ring rush
    drivetrain.boomerang(Pose(-15, 100, -25), 6000, 0.35, false, false, true);
    drivetrain.waitUntil(50);
    rushMech.setState(true);
    intake.setVoltage(12000);
    drivetrain.waitUntilEnd();
    drivetrain.stop(300);
    intake.setVoltage(1000);

    //Moving to mogo
    drivetrain.swingToHeading(-52, true, true, false, false);
    drivetrain.drive(-75, 12000, true);
    drivetrain.waitUntil(65);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    intake.setVoltage(12000);
    rushMech.setState(false);
    
    //Picking up rings
    drivetrain.turnToHeading(-87, 800, false, false);
    drivetrain.drive(80, 8000, false);
    drivetrain.swingToHeading(-120, true, true, false, false);
    drivetrain.boomerang(Pose(0, 13, 90), 6000, 0.5, false, false, false);
    
    //Shitssing teammate
    drivetrain.boomerang(Pose(110, 40, 90), 6000, 0.5, false, false, false);
    drivetrain.turnToHeading(-90, 2000, false, false);
    mogo.setState(false);
    drivetrain.drive(-50, 12000, false);
    drivetrain.turnToHeading(-90, 2000, false, false);
    
    //alliance stake
    drivetrain.boomerang(Pose(100, 10, -90), 6000, 0.5, false, false, false);
    drivetrain.turnToHeading(0, 800, false, false);

    //other mogo and ring, then ladder
    drivetrain.stop(300);
}

void opcontrol() 
{
    intake.setRingSortColour(true);
    drivetrain.stopOdom();
    drivetrain.setBrakeMode(MOTOR_BRAKE_COAST);
	while(true)
    {
        //Controlling Drivetrain
        drivetrain.driverFunctions();
        //Controlling Intake
        intake.driverFunctions();
        //Controlling Lift
        lift.driverFunctions();
        //Controlling Mogo
        mogo.driverFunctions();
        //Controlling Rush Mech
        rushMech.driverFunctions();
        //Controlling Doinker
        doinker.driverFunctions();
        //Controlling Pto
        pto.driverFunctions();
        
        pros::delay(10);
    }
}