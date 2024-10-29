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
    subsystems::intake intake = subsystems::intake(INTAKE, OPTICAL);
    //Creating lift
    subsystems::lift lift = subsystems::lift(LIFT);
    //Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO);
    //Creating arm
    subsystems::arms arm = subsystems::arms(ARM);


    //Auton paths and motion profiling
    cubicBezier curve1 = cubicBezier(Point(10, -85), Point(5, -60), Point(-10, -20), Point(-10, -20));
    profile motionProfile = profile(175.6, 473.4, 275);
    std::vector<std::vector<double>> profile1;

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
    drivetrain.runOdom(Pose(0, 0, 0));
    lift.holdPosition(subsystems::LiftPosition::DEFAULT);
    drivetrain.setBrakeMode(MOTOR_BRAKE_HOLD);

    

    drivetrain.stopOdom();
    lift.endHold();
}

void opcontrol() 
{
    //Colour we are
    bool colour = false;

    drivetrain.setBrakeMode(MOTOR_BRAKE_COAST);
    lift.holdPosition(subsystems::LiftPosition::DEFAULT);
	while(true)
    {
        //Controlling Drivetrain
        drivetrain.driverFunctions();
        //Controlling Intake
        intake.driverFunctions(colour);
        //Controlling Plunger
        lift.driverFunctions();
        //Controlling Mogo
        mogo.driverFunctions();
        //Controlling Arm
        arm.driverFunctions();
        
        pros::delay(10);
    }
}