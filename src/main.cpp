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
        INERTIAL,
        DIST_LEFT,
        DIST_RIGHT,
        DIST_BACK
    );

//Creating subsystems
    subsystems::intake intake = subsystems::intake(LOWER_INTAKE_1, LOWER_INTAKE_2, UPPER_INTAKE, HOOD);
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

void sawp()
{
    drivetrain.runOdom(Pose(190, 65, -90, -90));

        // sig sawp

        //Pushing teammate off line, and taking preload
            intake.setIntakeState(12000, 12000, true);
            drivetrain.moveToPoint(Point(170, 65), false, 500, false);
            drivetrain.boomerang(Pose(299, 65, 90, 90), 12000, 0.5, true, false, false, 1200);


        //Matchloading
            matchload.setState(true);
            drivetrain.turnToHeading(-180, false, 700);
            drivetrain.boomerang(Pose(306, 35, 180, 180), 6000, 0.5, false, false, false, 700);
            drivetrain.stop(600);

            //Distance reset
            drivetrain.distanceReset(0, 1);

        //Scoring long goal
            drivetrain.boomerang(Pose(305, 120, 0, 0), 12000, 0.5, true, false, false, 1100);
            intake.setIntakeState(12000, 12000, false);
            drivetrain.stop(1300);
            drivetrain.distanceReset(0, 1);

        //Picking up mid balls
            matchload.setState(false);
            drivetrain.turnToHeading(-90, true, 1000);

            //Ressetting position after turning on the long goal
            //116.7 y, 275 x
            drivetrain.pose.x = 275;
            drivetrain.pose.y = 116.7;

            intake.setIntakeState(12000, 12000, true);
            drivetrain.boomerang(Pose(235, 125, -90, -90), 12000, 0.5, false, false, true, 800);
            drivetrain.waitUntil(5);
            matchload.setState(true);
            drivetrain.waitUntilEnd();
            matchload.setState(false);
            drivetrain.boomerang(Pose(115, 125, -90, -90), 9000, 0.5, false, false, true, 1300);
            drivetrain.waitUntil(43);
            matchload.setState(true);
            drivetrain.waitUntilEnd();

        //Scoring mid goal
            drivetrain.turnToHeading(-135, false, 700);
            drivetrain.boomerang(Pose(150, 151.5, 45, 45), 12000, 0.5, true, false, false, 800);
            intake.setIntakeState(12000, -12000, true);
            pros::delay(850);
            intake.setIntakeState(12000, 12000, true);

        //Matchloading
            drivetrain.boomerang(Pose(60, 60, -135, -135), 12000, 0.5, false, false, false, 1200);
            drivetrain.turnToHeading(-180, false, 800);
            // drivetrain.boomerang(Pose(61, 30, -180, -180), 12000, 0.5, false, false, false, 800);

            // //Distance reset
            // drivetrain.distanceReset(1, 0);
        //Scoring Long goal
            drivetrain.boomerang(Pose(60, 125, 0, 0), 12000, 0.5, true, false, false, 1000);
            intake.setIntakeState(12000, 12000, false);
            drivetrain.stop(1000);
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
    // selector.run_auton();


    //Left side 9 ball, 6 in long, 3 in mid
    

    //Picking up triple

    //Picking up double

    //Scoring long goal

    //Matchloading

    //Scoring mid

    //Pushing long goal for control

}

void opcontrol() 
{
    // drivetrain.stopOdom();
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