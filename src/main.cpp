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
    subsystems::intake intake = subsystems::intake(LOWER_INTAKE_1, LOWER_INTAKE_2, UPPER_INTAKE, OPTICAL, HOOD);
    subsystems::matchload matchload = subsystems::matchload(MATCHLOAD);
    subsystems::descore descore = subsystems::descore(DESCORE);

void sawp()
{
    drivetrain.runOdom(Pose(190, 65, -90, -90));

        // 11 ball sig sawp, 5 in long, 2 in mid, 4 in other long, 61 points with all bonus

        //Pushing teammate off line, and taking preload
            intake.setIntakeState(12000, 12000, true);
            drivetrain.moveToPoint(Point(170, 65), false, 500, false);
            drivetrain.boomerang(Pose(299, 65, 90, 90), 12000, 0.5, true, false, false, 1200);


        //Matchloading
            matchload.setState(true);
            drivetrain.turnToHeading(-180, false, 700);
            drivetrain.boomerang(Pose(305.5, 35, 180, 180), 7000, 0.5, false, false, false, 700);
            drivetrain.stop(700);

            //Distance reset
            drivetrain.distanceReset(0, 1);

        //Scoring long goal
            drivetrain.boomerang(Pose(306, 120, 0, 0), 12000, 0.5, true, false, false, 1100);
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
            drivetrain.boomerang(Pose(149, 150.5, 45, 45), 12000, 0.5, true, false, false, 800);
            intake.setIntakeState(12000, -12000, true);
            pros::delay(900);
            intake.setIntakeState(12000, 12000, true);

        //Matchloading
            drivetrain.boomerang(Pose(56, 60, -135, -135), 12000, 0.5, false, false, false, 1200);
            drivetrain.turnToHeading(-180, false, 800);
            // drivetrain.boomerang(Pose(61, 30, -180, -180), 12000, 0.5, false, false, false, 800);

            // //Distance reset
            // drivetrain.distanceReset(1, 0);
        //Scoring Long goal
            drivetrain.boomerang(Pose(58, 125, 0, 0), 12000, 0.5, true, false, false, 1000);
            intake.setIntakeState(12000, 12000, false);
            drivetrain.stop(1000);
}

void leftLongMid9()
{
    //Left side 9 ball, 6 in long, 3 in mid, 45 points with both bonus
    drivetrain.runOdom(Pose(141.7, 61.7, 0, 0));


    //Picking up triple
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(118.4, 125, -45, -45), 7000, 0.5, false, false, false, 1000);

    //Picking up double
        drivetrain.boomerang(Pose(60, 151, -80, -80), 7000, 0.5, false, false, false, 1000);

    // //Scoring long goal

        //Resetting pos
        //y = 161.7, x = 81.7
        drivetrain.pose.x = 81.7;
        drivetrain.pose.y = 161.7;

        drivetrain.boomerang(Pose(120, 130, 135, 135), 12000, 0.5, true, false, false, 1000);
        drivetrain.turnToHeading(-135, false, 1000);
        drivetrain.boomerang(Pose(60, 60, -135, -135), 12000, 0.5, false, false, false, 1000);
        drivetrain.turnToHeading(-180, false, 1000);

        drivetrain.boomerang(Pose(63, 125, 0, 0), 12000, 0.5, true, false, false, 1000);
        intake.setIntakeState(12000, 12000, false);
        drivetrain.stop(1200);

        //Resetting pos
        drivetrain.distanceReset(1, 0);

    //Matchloading  
        matchload.setState(true);
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(62.5, 33, 180, 180), 8000, 0.5, false, false, false, 1000);
        drivetrain.stop(800);

        //Resetting pos
        drivetrain.distanceReset(1, 0);
        
    //Scoring mid
        drivetrain.boomerang(Pose(60, 60, 0, 0), 12000, 0.5, true, false, false, 700);
        drivetrain.turnToHeading(-135, true, 700);
        drivetrain.boomerang(Pose(153, 156, 45, 45), 12000, 0.5, true, false, false, 1300);
        intake.setIntakeState(12000, -12000, true);
        drivetrain.stop(1000);
        matchload.setState(false);

    // //Pushing long goal for control
        drivetrain.boomerang(Pose(30, 80, -115, -115), 12000, 0.5, false, false, false, 1500);
        drivetrain.turnToHeading(-180, false, 1000);
        descore.setState(true);
        drivetrain.boomerang(Pose(30, 160, 2, 2), 8000, 0.5, true, false, false, 1000);
}


void rightLongMid9()
{
    //Right side 9 ball, 6 in long, 3 in mid, 45 points with both bonus
    drivetrain.runOdom(Pose(224.06, 61.7, 0, 0));


    //Picking up triple
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(247.36, 125, 45, 45), 7000, 0.5, false, false, false, 1000);

    //Picking up double
        drivetrain.boomerang(Pose(305.76, 151, 80, 80), 7000, 0.5, false, false, false, 1000);

    // //Scoring long goal

        //Resetting pos
        //y = 161.7, x = 81.7
        drivetrain.pose.x = 284.06;
        drivetrain.pose.y = 161.7;

        drivetrain.boomerang(Pose(245.76, 130, -135, -135), 12000, 0.5, true, false, false, 1000);
        drivetrain.turnToHeading(135, true, 1000);
        drivetrain.boomerang(Pose(306, 60, 135, 135), 12000, 0.5, false, false, false, 1000);
        drivetrain.turnToHeading(180, true, 1000);

        drivetrain.boomerang(Pose(307, 125, 0, 0), 12000, 0.5, true, false, false, 1000);
        intake.setIntakeState(12000, 12000, false);
        drivetrain.stop(1200);

        //Resetting pos
        drivetrain.distanceReset(0, 1);

    //Matchloading  
        matchload.setState(true);
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(303.26, 39, 180, 180), 8000, 0.5, false, false, false, 1000);
        drivetrain.stop(800);

        //Resetting pos
        drivetrain.distanceReset(0, 1);
        
    //Scoring mid
        drivetrain.boomerang(Pose(305.76, 60, 0, 0), 12000, 0.5, true, false, false, 700);
        drivetrain.turnToHeading(-45, true, 700);
        matchload.setState(false);
        drivetrain.boomerang(Pose(208, 156, -45, -45), 12000, 0.5, false, false, false, 1500);
        intake.setIntakeState(-12000, -12000, true);
        drivetrain.stop(1000);

    // // //Pushing long goal for control
    //     drivetrain.boomerang(Pose(30, 80, -115, -115), 12000, 0.5, false, false, false, 1500);
    //     drivetrain.turnToHeading(-180, false, 1000);
    //     descore.setState(true);
    //     drivetrain.boomerang(Pose(30, 160, 2, 2), 8000, 0.5, true, false, false, 1000);
}

void leftLong9()
{
    //Left side 9 ball, 9 in long, 37 points with both bonus
    drivetrain.runOdom(Pose(141.7, 61.7, 0, 0));


    //Picking up triple
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(118.4, 125, -45, -45), 7000, 0.5, false, false, false, 1000);

    //Picking up double
        drivetrain.boomerang(Pose(60, 151, -80, -80), 6000, 0.5, false, false, false, 1000);

    //Scoring long goal

        //Resetting pos
        //y = 161.7, x = 81.7
        drivetrain.pose.x = 81.7;
        drivetrain.pose.y = 161.7;

        drivetrain.boomerang(Pose(120, 130, 135, 135), 12000, 0.5, true, false, false, 1000);
        drivetrain.turnToHeading(-135, false, 1000);
        drivetrain.boomerang(Pose(61, 60, -135, -135), 12000, 0.5, false, false, false, 1000);
        drivetrain.turnToHeading(-180, false, 1000);

        drivetrain.boomerang(Pose(61, 115, 0, 0), 12000, 0.5, true, false, false, 1000);
        intake.setIntakeState(12000, 12000, false);
        drivetrain.stop(1200);

        //Resetting pos
        drivetrain.distanceReset(1, 0);

    //Matchloading  
        matchload.setState(true);
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(61, 31, 180, 180), 8000, 0.5, false, false, false, 1000);
        drivetrain.stop(800);

        //Resetting pos
        drivetrain.distanceReset(1, 0);
        
    //Scoring Long
        drivetrain.boomerang(Pose(60, 60, 0, 0), 12000, 0.5, true, false, false, 700);
        drivetrain.turnToHeading(-135, true, 700);
        drivetrain.boomerang(Pose(152.5, 153.5, 45, 45), 12000, 0.5, true, false, false, 1300);
        intake.setIntakeState(12000, -12000, true);
        drivetrain.stop(1000);
        matchload.setState(false);
}

void rightLong9()
{

}

void skills()
{
    //96 auto,
    //12 in both long goals with bonus
    //7 in to low goal with bonus
    //Both parks cleared and parking
    //All loaders cleared

    // drivetrain.runOdom(Pose(0, 0, 0, 0));

    // //Getting balls from park zone
    // intake.setIntakeState(12000, 12000, true);
    // drivetrain.drive(20, 0, 127);
    // pros::delay(2000);
    // drivetrain.drive(30, 0, 127);

    //Scoring low
    //Lift intake and slow speed for low goal


    drivetrain.runOdom(Pose(60, 60, 180, 180));
    descore.setState(false);

    //Clearing 1st loader
        matchload.setState(true);
        intake.setIntakeState(12000, 12000, true);
        pros::delay(600);
        drivetrain.boomerang(Pose(60, 33.5, 180, 180), 6000, 0.5, false, false, false, 1000);
        drivetrain.stop(1800);
        drivetrain.pose.x = 60;

        drivetrain.boomerang(Pose(60, 60, 0, 0), 12000, 0.5, true, false, false, 700);

    //Scoring 1st long
        drivetrain.boomerang(Pose(30, 90, -45, -45), 12000, 0.5, true, false, false, 1000);
        matchload.setState(false);
        drivetrain.boomerang(Pose(30, 240, 0, 0), 12000, 0.5, true, false, false, 1500);
        drivetrain.boomerang(Pose(64, 300, 45, 45), 12000, 0.5, true, false, false, 1000);

        drivetrain.turnToHeading(0, true, 1000);

        drivetrain.boomerang(Pose(63.5, 230, 180, 180), 12000, 0.5, true, false, false, 1000);
        intake.setIntakeState(12000, 12000, false);
        matchload.setState(true);
        drivetrain.stop(2200);
        drivetrain.pose.x = 60;
        drivetrain.pose.y = 265;

    //Clearing 2nd loader
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(62, 341, 0, 0), 5000, 0.5, false, false, false, 1500);
        drivetrain.stop(1800);
    
    //Scoring 1st long again
        drivetrain.boomerang(Pose(61, 238, 180, 180), 12000, 0.5, true, false, false, 1000);
        intake.setIntakeState(12000, 12000, false);
        matchload.setState(false);
        drivetrain.stop(2200);
        drivetrain.pose.x = 60;
        drivetrain.pose.y = 265;

    //Clearing 2nd park zone
        drivetrain.boomerang(Pose(250, 310, 90, 90), 12000, 0.5, false, false, false, 2000);
        drivetrain.boomerang(Pose(210, 350, -65, -65), 12000, 0.5, false, false, false, 2000);
        drivetrain.turnToHeading(-90, false, 1000);

        matchload.setState(true);
        intake.setIntakeState(12000, 12000, true);
        pros::delay(400);

        drivetrain.drive(95, -88, 127);
        drivetrain.drive(-90, 90, 127);
        matchload.setState(false);
        pros::delay(300);

        drivetrain.drive(60, -90*M_PI/180, 1100, 25, 1, false);
        drivetrain.pose.x = 223;
        drivetrain.pose.y = 345;

    //Clearing 3rd matchload
        drivetrain.boomerang(Pose(295, 290, 135, 135), 12000, 0.2, true, false, false, 2000);
        matchload.setState(true);
        drivetrain.turnToHeading(0, true, 1000);
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(295, 333.5, 0, 0), 5000, 0.2, false, false, false, 1500);
        drivetrain.stop(1800);
        drivetrain.pose.x = 300;

        drivetrain.boomerang(Pose(300, 300, 180, 180), 12000, 0.5, true, false, false, 1000);

    //Scoring 2nd long
        drivetrain.boomerang(Pose(335, 270, 135, 135), 12000, 0.5, true, false, false, 1000);
        matchload.setState(false);
        drivetrain.boomerang(Pose(335, 120, 180, 180), 12000, 0.5, true, false, false, 2000);
        drivetrain.boomerang(Pose(304, 60, -135, -135), 12000, 0.5, true, false, false, 2000);
        drivetrain.turnToHeading(180, true, 1000);

        drivetrain.boomerang(Pose(304, 130, 0, 0), 12000, 0.5, true, false, false, 1000);
        intake.setIntakeState(12000, 12000, false);
        matchload.setState(true);
        drivetrain.stop(2200);
        drivetrain.pose.x = 300;
        drivetrain.pose.y = 105;

    //Clearing 4th matchload
        intake.setIntakeState(12000, 12000, true);
        drivetrain.boomerang(Pose(298.5, 31.5, 180, 180), 5000, 0.5, false, false, false, 1500);
        drivetrain.stop(1800);


    //Scoring 2nd long again
        drivetrain.boomerang(Pose(301.5, 130, 0, 0), 12000, 0.5, true, false, false, 1000);
        intake.setIntakeState(12000, 12000, false);
        matchload.setState(false);
        drivetrain.stop(2200);
        drivetrain.pose.x = 300;
        drivetrain.pose.y = 105;

    //Parking
        drivetrain.boomerang(Pose(240, 30, -135, -135), 12000, 0.5, false, false, false, 2000);
        drivetrain.boomerang(Pose(220, 20, -90, -90), 12000, 0.5, false, false, false, 1000);
        drivetrain.drive(80, -88, 127);
        drivetrain.stop(500);
}

rd::Selector selector {{
    {"Sawp", sawp},
    {"Right long 9", rightLong9},
    {"Right long mid 9", rightLongMid9},
    {"Left long 9", leftLong9},
    {"Left long mid 9", leftLongMid9},
    {"Skills", skills}
}};

void initialize()
{
    pros::IMU imu = pros::IMU(INERTIAL);

    imu.reset();

    descore.setState(true);
}

void autonomous()
{
    // selector.run_auton();
    skills();
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