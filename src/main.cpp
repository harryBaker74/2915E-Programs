#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/velocityController.hpp"
#include "../include/harryLibHeader/pathGen.hpp"
#include "../include/harryLibHeader/odom.hpp"

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

//Creating subsystems
    subsystems::intake intake = subsystems::intake(LOWER_INTAKE, UPPER_INTAKE, HOOD);
    subsystems::matchload matchload = subsystems::matchload(MATCHLOAD);
    subsystems::descore descore = subsystems::descore(DESCORE);

void rightLongGoal()
{
    // drivetrain.runOdom(Pose(225, 52, 0, 0));
    // matchload.setState(0, 1);
    // drivetrain.drive(38, 0, 127);
    // drivetrain.turnToHeading(25, true, 1000);
    // intake.setIntakeState(12000, 12000, 12000);
    // drivetrain.drive(40, 25, 40);
    // drivetrain.drive(-5, -155);
    
    // drivetrain.turnToHeading(-45, false, 1000);
    // drivetrain.drive(-90, 135);
    // matchload.setState(0, 0);
    // drivetrain.turnToHeading(0, true, 1000);
    // intake.setIntakeState(-12000, 12000, 12000);
    // drivetrain.drive(45, 0, 1000, 100, 1, false);
    // pros::delay(3000);

    // drivetrain.drive(-10, 180, 127);
}

void rightLongMidGoal()
{
    // drivetrain.runOdom(Pose(225, 52, 0, 0));
    // matchload.setState(0, 1);
    // drivetrain.drive(38, 0, 127);
    // drivetrain.turnToHeading(25, true, 1000);
    // intake.setIntakeState(12000, 12000, 12000);
    // drivetrain.drive(40, 25, 40);
    // drivetrain.drive(-5, -155);
    
    // drivetrain.turnToHeading(-45, false, 1000);
    // drivetrain.drive(-90, 135);
    // matchload.setState(0, 0);
    // drivetrain.turnToHeading(0, true, 1000);
    // intake.setIntakeState(-12000, 12000, 12000);
    // drivetrain.drive(45, 0, 1000, 100, 1, false);
    // pros::delay(1800);

    // drivetrain.drive(-10, 180, 127);
    // intake.setIntakeState(12000, 12000, 0);
    // drivetrain.turnToHeading(-90, false, 1000);
    // drivetrain.drive(142, -90, 127);
    // drivetrain.turnToHeading(-45, true, 1000);
    // drivetrain.drive(49, -45, 40);
    // drivetrain.turnToHeading(45, true, 1000);
    // matchload.setState(1, 1);
    // intake.setIntakeState(-12000, 12000, -8000);
    // drivetrain.drive(35, 45, 127);
}

void leftMidLongGoal()
{
    // drivetrain.runOdom(Pose(135, 52, 0, 0));
    // matchload.setState(0, 1);

    // drivetrain.drive(38, 0, 127);
    // drivetrain.turnToHeading(-25, false, 1000);
    // intake.setIntakeState(12000, 12000, 12000);
    // drivetrain.drive(34, -25, 24);
    // drivetrain.turnToHeading(45, true, 1000);
    // matchload.setState(1, 1);
    // drivetrain.drive(30, 45);
    // intake.setIntakeState(-12000, 12000, -5000);
    // pros::delay(2500);
    // drivetrain.drive(10, 45*M_PI/180, 400, 50, 1, false);
    
    // intake.setIntakeState(12000, 12000, -12000);
    // drivetrain.drive(-20, -135, 127);
    // drivetrain.turnToHeading(-70, false, 1000);
    // matchload.setState(0, 0);
    // pros::delay(500);
    // drivetrain.drive(60, -70, 127);
    // pros::delay(500);

    // drivetrain.drive(-59, 110, 127);
    // matchload.setState(1, 1);
    // pros::delay(300);
    // drivetrain.turnToHeading(45, true, 1000);
    // drivetrain.drive(5, 45, 127);
    // intake.setIntakeState(-12000, 12000, -12000);
    // pros::delay(1500);
    // drivetrain.drive(5, 45, 127);
    // drivetrain.drive(-15, -135, 127);
}

void rightLowGoal()
{
    drivetrain.runOdom(Pose(0, 0, 0, 0));
    descore.setState(true);
    drivetrain.drive(60, 0, 100);
    drivetrain.turnToHeading(22, true, 1000);
    intake.setIntakeState(12000, 12000, false);
    drivetrain.drive(70, 22, 30);
    pros::delay(500);
    drivetrain.drive(-24, -156, 50);
    drivetrain.turnToHeading(-45, false, 1000);
    drivetrain.drive(42, -45 * M_PI/180, 1500, 40, 1, false);
    intake.setIntakeState(-12000, -12000, false);
    // pros::delay(500);
    // drivetrain.drive(5, -45, 30);
    pros::delay(2000);
    intake.setIntakeState(12000, 12000, false);
    pros::delay(500);
    intake.setIntakeState(-12000, -12000, false);
    pros::delay(2000);
    intake.setIntakeState(12000, 12000, false);
    pros::delay(500);
    intake.setIntakeState(-12000, -12000, false);
}

void leftMidGoal()
{
    drivetrain.runOdom(Pose(0, 0, 0, 0));
    drivetrain.drive(60, 0, 100);
    drivetrain.turnToHeading(-22, false, 1000);
    intake.setIntakeState(12000, 0, false);
    drivetrain.drive(70, -22, 30);
    pros::delay(500);
    drivetrain.drive(-25, 156, 50);
    drivetrain.turnToHeading(45, true, 1000);
    intake.setIntakeState(0, 0, true);
    drivetrain.drive(48, 45 * M_PI/180, 1500, 40, 1, false);
    intake.setIntakeState(12000, 12000, true);
}

//8 ball for right side long goal
void rightLong8()
{
    drivetrain.runOdom(Pose(0, 0, 0, 0));
    //Getting 3
    intake.setIntakeState(12000, 12000, false);
    drivetrain.drive(35, 0, 100);
    drivetrain.turnToHeading(38, true, 600);
    drivetrain.drive(65, 38, 40);

    //Getting 1 under
    drivetrain.turnToHeading(70, true, 600);
    drivetrain.drive(32, 70*M_PI/180, 1000, 80, 1, false);

    //Going to long goal
    drivetrain.drive(-47, -110, 100);
    drivetrain.turnToHeading(135, true, 1000);
    drivetrain.drive(84, 135, 127);
    drivetrain.turnToHeading(180, true, 600);
    drivetrain.drive(-28, 0, 500, 127, 1, false);
    intake.setIntakeState(12000, 12000, true);
    pros::delay(400);
    intake.setIntakeState(12000, 12000, true);
    pros::delay(1750);

    //Going to Matchload
    matchload.setState(true);
    intake.setIntakeState(12000, -12000, false);
    drivetrain.drive(85, -176*M_PI/180, 1600, 60, 0.1, false);

    //Back to long goal
    drivetrain.drive(-75, 2, 127);
    intake.setIntakeState(12000, 12000, true);
    pros::delay(1900);
    
    //Smashing into the goal
    drivetrain.drive(20, 180, 127);
    intake.setIntakeState(12000, 12000, false);
    pros::delay(300);
    drivetrain.drive(-50, 0, 127);
}

//8 ball for left side long goal
void leftLong8()
{
    drivetrain.runOdom(Pose(0, 0, 0, 0));
    //Getting 3
    intake.setIntakeState(12000, 12000, false);
    drivetrain.drive(35, 0, 100);
    drivetrain.turnToHeading(-38, false, 600);
    drivetrain.drive(65, -38, 40);

    //Getting 1 under
    drivetrain.turnToHeading(-70, false, 600);
    drivetrain.drive(30, -70*M_PI/180, 1000, 80, 1, false);

    //Going to long goal
    drivetrain.drive(-47, 110, 100);
    drivetrain.turnToHeading(-135, false, 1000);
    drivetrain.drive(84, -135, 127);
    drivetrain.turnToHeading(-180, false, 600);
    drivetrain.drive(-28, 0, 500, 127, 1, false);
    intake.setIntakeState(12000, 12000, true);
    pros::delay(400);
    intake.setIntakeState(12000, 12000, true);
    pros::delay(1750);

    //Going to Matchload
    matchload.setState(true);
    intake.setIntakeState(12000, 12000, false);
    drivetrain.drive(80, 176*M_PI/180, 1600, 60, 0.1, false);

    //Back to long goal
    drivetrain.drive(-75, -3, 127);
    intake.setIntakeState(12000, 12000, true);
    pros::delay(1900);
    
    //Smashing into the goal
    drivetrain.drive(20, -180, 127);
    intake.setIntakeState(12000, 12000, false);
    pros::delay(300);
    drivetrain.drive(-50, 0, 127);
}

rd::Selector selector {{
    {"Right long Goal", rightLong8},
    {"Left long Goal", leftLong8}
}};

void initialize()
{
    pros::IMU imu = pros::IMU(INERTIAL);

    imu.reset();
}

void autonomous()
{
    // leftMidLongGoal();
    selector.run_auton();
    // rightLong8();
    // leftLong8();

    // drivetrain.runOdom(Pose(0, 0, 0, 0));
    // intake.setIntakeState(12000, 12000, false);

    // //Pick up 3
    // drivetrain.moveToPoint(Point(0, 30), false, 1000, false);
    // drivetrain.stop(100);
    // drivetrain.turnToHeading(35, true, 1000);
    // drivetrain.moveToPoint(Point(50, 90), false, 1000, false);
    // drivetrain.stop(200);

    // //Long goal
    // drivetrain.turnToHeading(0, false, 1000);
    // drivetrain.moveToPoint(Point(60, 20), true, 1000, false);
    // drivetrain.stop(200);
    // drivetrain.turnToHeading(90, true, 1000);
    // drivetrain.moveToPoint(Point(90, 10), false, 1000, false);
    // drivetrain.stop(200);
    // drivetrain.turnToHeading(180, true, 1000);
    // drivetrain.moveToPoint(Point(90, 60), true, 1000, false);
    // intake.setIntakeState(12000, 12000, true);
    // drivetrain.stop(200);
    // pros::delay(1000);


    // //Matchload
    // matchload.setState(true);
    // intake.setIntakeState(12000, 12000, false);
    // drivetrain.moveToPoint(Point(87, -20), false, 1000, false);
    // drivetrain.stop(200);
    // pros::delay(500);
    

    // //Long goal
    // drivetrain.moveToPoint(Point(90, 60), true, 1000, false);
    // intake.setIntakeState(12000, 12000, true);

    // drivetrain.stop(200);
    // matchload.setState(false);
    // pros::delay(1000);

    // //Descore
    // drivetrain.moveToPoint(Point(90, 20), false, 1000, false);
    // drivetrain.stop(200);
    // drivetrain.turnToHeading(135, false, 1000);
    // drivetrain.moveToPoint(Point(60, 40), true, 1000, false);
    // drivetrain.stop(200);
    // descore.setState(false);
    // drivetrain.turnToHeading(180, true, 1000);
    // drivetrain.moveToPoint(Point(60, 100), true, 1000, false);
    // drivetrain.stop(200);

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
        //Matchloader
        matchload.driverFunctions();
        //Descore
        descore.driverFunctions();

        pros::delay(10);
    }
}