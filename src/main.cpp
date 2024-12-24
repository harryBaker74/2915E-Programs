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
    subsystems::intake intake = subsystems::intake(INTAKE);
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
    drivetrain.setBrakeMode(MOTOR_BRAKE_HOLD);

    /*drivetrain.moveToPoint(Point(0, -70), true, true);
    drivetrain.waitUntil(60);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    intake.setVoltage(12000);

    drivetrain.moveToPoint(Point(-70, -60), false, false);
    

    drivetrain.moveToPoint(Point(-70, -106), false, false);

    drivetrain.moveToPoint(Point(-50, -70), true, false);
    drivetrain.moveToPoint(Point(-50, -110), false, false);

    drivetrain.moveToPoint(Point(-60, -70), true, false);

    drivetrain.moveToPoint(Point(-60, -30), false, false);

    drivetrain.boomerang(Pose(-100, 15, -45), 7000, 0.5, false, false, false);
    
    pros::delay(300);
    drivetrain.moveToPoint(Point(45, -10), true, true);
    pros::delay(500);
    intake.holdPosition(subsystems::LiftPosition::ALLIANCE);
    drivetrain.waitUntilEnd();

    drivetrain.stop(300);*/

    drivetrain.drive(-75, 6000, true);
    drivetrain.waitUntil(74);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    intake.setVoltage(12000);

    drivetrain.moveToPoint(Point(60, -70), false, false);
    
    drivetrain.moveToPoint(Point(70, -90), false, false);

    drivetrain.moveToPoint(Point(70, -112), false, false);

    drivetrain.moveToPoint(Point(25, -70), true, false);
    drivetrain.moveToPoint(Point(42, -113), false, false);

    drivetrain.moveToPoint(Point(60, -70), true, false);

    drivetrain.moveToPoint(Point(60, -30), false, false);

    drivetrain.stop();




    //Wp 4 ring side, blue team
    
    /*drivetrain.drive(-75, 6000, true);
    drivetrain.waitUntil(74);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    intake.setVoltage(12000);

    drivetrain.moveToPoint(Point(-75, -72), false, false);
    
    drivetrain.moveToPoint(Point(-70, -72), false, false);

    drivetrain.moveToPoint(Point(-70, -111), false, false);

    drivetrain.moveToPoint(Point(-20, -72), true, false);
    drivetrain.moveToPoint(Point(-35, -113), false, false);

    drivetrain.moveToPoint(Point(-60, -72), true, false);

    drivetrain.moveToPoint(Point(-60, -32), false, false);

    drivetrain.stop();
    */

    //Elim 4 ring side, blue team
    /*
    drivetrain.drive(-70, 6000, true);
    drivetrain.waitUntil(65);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    intake.setVoltage(12000);

    drivetrain.moveToPoint(Point(-75, -70), false, false);
    
    drivetrain.moveToPoint(Point(-70, -70), false, false);

    drivetrain.moveToPoint(Point(-70, -112), false, false);

    drivetrain.moveToPoint(Point(-20, -70), true, false);
    drivetrain.moveToPoint(Point(-35, -112), false, false);

    drivetrain.moveToPoint(Point(-60, -70), true, false);

    drivetrain.moveToPoint(Point(-60, -30), false, false);

    drivetrain.boomerang(Pose(-98, 13, -45), 7000, 0.5, false, false, true);
    drivetrain.waitUntil(60);
    intake.setVoltage(0);
    drivetrain.waitUntilEnd();

    pros::delay(300);
    drivetrain.moveToPoint(Point(45, -10), true, false);

    drivetrain.setBrakeMode(MOTOR_BRAKE_COAST);
    drivetrain.setVoltage(0, 0);
    pros::delay(200);
    intake.setVoltage(12000);
    */

}

void opcontrol() 
{
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