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
    //lift.holdPosition(subsystems::LiftPosition::DEFAULT);
    drivetrain.setBrakeMode(MOTOR_BRAKE_HOLD);

    
    //4 ring, ring side, awp end
    /*drivetrain.drive(-27, 12000, false);
    drivetrain.swingToHeading(-30, true, true, false, false);
    drivetrain.drive(-52, 6000, false);
    mogo.setState(true);
    pros::delay(200);
    intake.setVoltage(12000);
    drivetrain.drive(-5, 12000, false);
    drivetrain.turnToHeading(-90, 2000, false, false);
    drivetrain.drive(45, 12000, false);
    pros::delay(650);
    drivetrain.turnToHeading(-180, 2500, false, false);
    drivetrain.drive(31, 12000, false);
    drivetrain.drive(-38, 12000, false);
    drivetrain.swingToHeading(-155, true, false, false, false);
    drivetrain.drive(30, 12000, false);
    drivetrain.stop(300);
    drivetrain.drive(-40, 12000, false);
    drivetrain.swingToHeading(90, false, false, false, false);
    drivetrain.drive(50, 12000, false);
    */

    //4 ring, ring side, elim end
    drivetrain.drive(-27, 12000, false);
    drivetrain.swingToHeading(-30, true, true, false, false);
    drivetrain.drive(-52, 6000, false);
    mogo.setState(true);
    pros::delay(200);
    intake.setVoltage(12000);
    drivetrain.drive(-5, 12000, false);
    drivetrain.turnToHeading(-90, 1600, false, false);
    drivetrain.drive(45, 12000, false);
    pros::delay(650);
    drivetrain.turnToHeading(-180, 2300, false, false);
    drivetrain.drive(31, 12000, false);
    drivetrain.drive(-38, 12000, false);
    drivetrain.swingToHeading(-155, true, false, false, false);
    drivetrain.drive(30, 12000, false);
    drivetrain.stop(300);
    drivetrain.drive(-40, 12000, false);
    drivetrain.swingToHeading(-260, true, true, false, false);
    drivetrain.stop(200);
    intake.setVoltage(-12000);
    drivetrain.drive(200, 12000, false);

    drivetrain.stop(200);
}

void opcontrol() 
{
    //Colour we are
    bool colour = false;

    //drivetrain.stopOdom();
    lift.endHold();
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