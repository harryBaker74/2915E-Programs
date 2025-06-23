#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/velocityController.hpp"
#include "../include/harryLibHeader/pathGen.hpp"
#include "../include/harryLibHeader/odom.hpp"

void redNeg_6_1_Qual();
void redNeg_6_1_Elim();
void redNeg_5_1_Qual();
void redNeg_5_1_Elim();
void redNeg_6_Qual();
void redNeg_6_Elim();

void redPos_6_Qual();
void redPos_6_Elim();
void redPos_1_4_Qual();
void redPos_1_4_Elim();
void redSawp_1_4_1_Qual();


void blueNeg_6_1_Qual();
void blueNeg_6_1_Elim();
void blueNeg_5_1_Qual();
void blueNeg_5_1_Elim();
void blueNeg_6_Qual();
void blueNeg_6_Elim();

void bluePos_6_Qual();
void bluePos_6_Elim();
void bluePos_1_4_Qual();
void bluePos_1_4_Elim();
void blueSawp_1_4_1_Qual();

void testAuto();


//Auton select
rd::Selector selector {{
    {"Red Neg 6+1 Qual", redNeg_6_1_Qual},
    {"Red Neg 6+1 Elim", redNeg_6_1_Elim},
    {"Red Neg 5+1 Qual", redNeg_5_1_Qual},
    {"Red Neg 5+1 Elim", redNeg_5_1_Elim},
    {"Red Neg 6 Qual", redNeg_6_Qual},
    {"Red Neg 6 Elim", redNeg_6_Elim},
    
    {"Red Pos 6 Qual", redPos_6_Qual},
    {"Red Pos 6 Elim", redPos_6_Elim},
    {"Red Pos 1+4 Qual", redPos_1_4_Qual},
    {"Red Pos 1+4 Elim", redPos_1_4_Elim},
    {"Red 1+4+1 Sawp", redSawp_1_4_1_Qual},

    {"Blue Neg 6+1 Qual", blueNeg_6_1_Qual},
    {"Blue Neg 6+1 Elim", blueNeg_6_1_Elim},
    {"Blue Neg 5+1 Qual", blueNeg_5_1_Qual},
    {"Blue Neg 5+1 Elim", blueNeg_5_1_Elim},
    {"Blue Neg 6 Qual", blueNeg_6_Qual},
    {"Blue Neg 6 Elim", blueNeg_6_Elim},
    
    {"Blue Pos 6 Qual", bluePos_6_Qual},
    {"Blue Pos 6 Elim", bluePos_6_Elim},
    {"Blue Pos 1+4 Qual", bluePos_1_4_Qual},
    {"Blue Pos 1+4 Elim", bluePos_1_4_Elim},
    {"Blue 1+4+1 Sawp", blueSawp_1_4_1_Qual},

    {"Testing", testAuto}
}};

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
    subsystems::intake intake = subsystems::intake(INTAKE, INTAKE_OPTICAL, INTAKE_LIFT, LIFT, ROTATION, LIFT_OPTICAL);
//Creating Mogo
    subsystems::mogo mogo = subsystems::mogo(MOGO, MOGO_OPTICAL);
//Creating Doinkers
    subsystems::doinkers doinkers = subsystems::doinkers(LEFT_DOINKER, RIGHT_DOINKER);
//Creating goal grabber
    subsystems::goalGrabber goalGrabber = subsystems::goalGrabber(GOAL_GRABBER);

//Auton paths and motion profiling
profile motionProfile = profile(100, 50, 50);

Pose startPose(0, 0, 0, 0);



void initialize() 
{
	pros::IMU imu = pros::IMU(INERTIAL);
    imu.reset(false);

    //Generating mcl particles
    // Odometery::MCLGenerateParticles(startPose, 0.2, 500);
}

//Autons
    //Neg autons(6)
        //6+1, qual end, 11 points
        //6+1, elim end(no end really), 11 points

        //5+1, qual end, 10 points
        //5+1, elim end(clear corner), 10 points

        //6, qual end, 8 points
        //6, elim end(clear corner), 8 points
    //Pos autons(4)
        //6, qual end, 8 points
        //6, elim end(next to third), 8 points

        //1 + 4, qual end, 9 points
        //1 + 4, elim end(next to third), 9 points
    //Win point(1)
        //1 + 4 + 1, start neg side, qual end, 12 points

//11 autons, 22 total for both teams

void testAuto()
{
    intake.setLoadStartingPosition();  
}

//Red
    //Neg
    void redNeg_6_1_Qual()
    {
        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(true);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        drivetrain.waitUntil(43);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(85, 161, -45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(40, 162.5, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-33, -34, -135, -135), 7000, 0.5, false, false, false, 2100);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 7, drivetrain.pose.y + 7, 45, 45), 3500, 0.1, true, false, false, 800);
        intake.setIntakeLiftState(true);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(-27, -28, -135, -135), 8000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(125, 70, 90, 90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(175, 70, 90, 90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(150, 70, -90, -90), 12000, 0.5, true, false, false);
        
        // Qual end
        intake.holdPosition(subsystems::WALL);
        drivetrain.boomerang(Pose(155, 130, 0, 0), 12000, 0.3, false, false, false, 1000);
        drivetrain.stop(100);
    }

    void redNeg_6_1_Elim()
    {
        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(true);

        // //Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        // intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        // drivetrain.boomerang(Pose(124, 132, 20, 20), 6000, 0.4, true, false, true);
        // drivetrain.waitUntil(43);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(93, 163, -45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(45, 163, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-24, -10, -135, -135), 6000, 0.5, false, false, false, 1750);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(drivetrain.pose.x + 7, drivetrain.pose.y + 7, 45, 45), 3500, 0.1, true, false, false, 800);
        // intake.setIntakeLiftState(true);
        // drivetrain.stop(300);
        // drivetrain.boomerang(Pose(-24, -10, -135, -135), 8000, 0.1, false, false, false, 700);
        // intake.setIntakeLiftState(false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(110, 60, 90, 90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(170, 60, 90, 90), 4000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(150, 60, -90, -90), 12000, 0.5, true, false, false);

        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(true);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        drivetrain.waitUntil(43);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(85, 161, -45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(40, 162.5, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-33, -34, -135, -135), 7000, 0.5, false, false, false, 2100);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 7, drivetrain.pose.y + 7, 45, 45), 3500, 0.1, true, false, false, 800);
        intake.setIntakeLiftState(true);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(-27, -28, -135, -135), 8000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(125, 65, 90, 90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(175, 65, 90, 90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(150, 65, -90, -90), 12000, 0.5, true, false, false);
        drivetrain.stop(100);
    }

    void redNeg_5_1_Qual()
    {
        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(true);

        // //Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        // intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        // drivetrain.boomerang(Pose(124, 132, 20, 20), 6000, 0.4, true, false, true);
        // drivetrain.waitUntil(43);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(93, 163, -45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(45, 163, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-24, -10, -135, -135), 6000, 0.5, false, false, false, 1750);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(110, 65, 90, 90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(170, 65, 90, 90), 3000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(150, 65, -90, -90), 12000, 0.5, true, false, false);

        // // Qual end
        // intake.holdPosition(subsystems::WALL);
        // drivetrain.boomerang(Pose(150, 130, 0, 0), 12000, 0.3, false, false, false, 1000);
        // drivetrain.stop(100);

        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(true);

        // //Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        // intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        // drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        // drivetrain.waitUntil(43);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(85, 159, -45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(40, 159, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(43, 120, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-33, -54, -135, -135), 6000, 0.5, false, false, false, 2100);
        // drivetrain.stop(500);
        // drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(125, 70, 90, 90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(175, 70, 90, 90), 4000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(150, 70, -90, -90), 12000, 0.5, true, false, false);
        
        // // Qual end
        // intake.holdPosition(subsystems::WALL);
        // drivetrain.boomerang(Pose(155, 130, 0, 0), 12000, 0.3, false, false, false, 1000);
        // drivetrain.stop(100);

        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(true);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        drivetrain.waitUntil(43);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(85, 161, -45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(40, 162.5, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-33, -34, -135, -135), 7000, 0.5, false, false, false, 2100);
        drivetrain.stop(400);
        drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(125, 75, 90, 90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(175, 75, 90, 90), 3000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(150, 75, -90, -90), 12000, 0.5, true, false, false);
        
        // Qual end
        intake.holdPosition(subsystems::ALLIANCE);
        drivetrain.boomerang(Pose(155, 140, 20, 20), 12000, 0.3, false, false, false, 1200);
        drivetrain.stop(100);

    }

    void redNeg_5_1_Elim()
    {
        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(true);

        // //Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        // intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        // drivetrain.boomerang(Pose(124, 132, 20, 20), 6000, 0.4, true, false, true);
        // drivetrain.waitUntil(43);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(93, 163, -45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(45, 163, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-24, -10, -135, -135), 6000, 0.5, false, false, false, 1750);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(110, 65, 90, 90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(170, 65, 90, 90), 3000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(150, 65, -90, -90), 12000, 0.5, true, false, false);

        // // Elim end
        // drivetrain.boomerang(Pose(308, 0, 90, 90), 12000, 0.3, false, false, true, 2000);
        // drivetrain.waitUntil(10);
        // doinkers.setStates(false, true);
        // drivetrain.waitUntilEnd();
        // drivetrain.stop(200);
        // drivetrain.turnToHeading(-45, 1500, false, false);

        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(true);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        drivetrain.waitUntil(43);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(85, 161, -45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(40, 162.5, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-33, -34, -135, -135), 7000, 0.5, false, false, false, 2100);
        drivetrain.stop(400);
        drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(125, 75, 90, 90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(175, 75, 90, 90), 3000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(150, 75, -90, -90), 12000, 0.5, true, false, false);
        
        // // Elim end
        drivetrain.boomerang(Pose(308, 5, 90, 90), 12000, 0.3, false, false, true, 2000);
        drivetrain.waitUntil(10);
        doinkers.setStates(false, true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(200);
        drivetrain.turnToHeading(-45, 1500, false, false);
    }

    void redNeg_6_Qual()
    {
        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setRingSortColour(true);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        // drivetrain.boomerang(Pose(124, 132, 20, 20), 6000, 0.4, true, false, true);
        // drivetrain.waitUntil(43);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(93, 163, -45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(45, 163, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-24, -10, -135, -135), 6000, 0.5, false, false, false, 1750);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(110, 65, 90, 90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(170, 65, 90, 90), 3000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(150, 65, -90, -90), 12000, 0.5, true, false, false);

        // // Qual end
        // intake.holdPosition(subsystems::WALL);
        // drivetrain.boomerang(Pose(150, 130, 0, 0), 12000, 0.3, false, false, false, 1000);
        // drivetrain.stop(100);

        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setRingSortColour(true);

        // //Mogo
        // drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        // drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        // drivetrain.waitUntil(43);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(85, 161, -45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(40, 162.5, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-33, -34, -135, -135), 7000, 0.5, false, false, false, 2100);
        // drivetrain.stop(400);
        // drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(125, 75, 90, 90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(175, 75, 90, 90), 3000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(150, 75, -90, -90), 12000, 0.5, true, false, false);

        // // Qual end
        // intake.holdPosition(subsystems::WALL);
        // drivetrain.boomerang(Pose(155, 130, 0, 0), 12000, 0.3, false, false, false, 1000);
        // drivetrain.stop(100);



        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setRingSortColour(true);

        //Mogo
        drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        drivetrain.waitUntil(43);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(85, 161, -45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(40, 162.5, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-33, -34, -135, -135), 7000, 0.5, false, false, false, 2100);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 7, drivetrain.pose.y + 7, 45, 45), 3500, 0.1, true, false, false, 800);
        intake.setIntakeLiftState(true);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(-27, -28, -135, -135), 8000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(125, 70, 90, 90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(175, 70, 90, 90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(150, 70, -90, -90), 12000, 0.5, true, false, false);
        
        // Qual end
        intake.holdPosition(subsystems::WALL);
        drivetrain.boomerang(Pose(155, 130, 0, 0), 12000, 0.3, false, false, false, 1000);
        drivetrain.stop(100);
    }

    void redNeg_6_Elim()
    {   
        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setRingSortColour(true);
        // intake.holdPosition(subsystems::DEFAULT);

        // //Mogo
        // drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        // drivetrain.boomerang(Pose(124, 132, 20, 20), 6000, 0.4, true, false, true);
        // drivetrain.waitUntil(43);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(93, 163, -45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(45, 163, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-24, -10, -135, -135), 6000, 0.5, false, false, false, 1750);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(110, 65, 90, 90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(170, 65, 90, 90), 3000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(150, 65, -90, -90), 12000, 0.5, true, false, false);

        // // Elim end
        // drivetrain.boomerang(Pose(308, 0, 90, 90), 12000, 0.3, false, false, true, 2000);
        // drivetrain.waitUntil(10);
        // doinkers.setStates(false, true);
        // drivetrain.waitUntilEnd();
        // drivetrain.stop(200);
        // drivetrain.turnToHeading(-45, 1500, false, false);

        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setRingSortColour(true);

        //Mogo
        drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        drivetrain.boomerang(Pose(124, 132, 20, 20), 6500, 0.4, true, false, true);
        drivetrain.waitUntil(43);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(85, 161, -45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(40, 162.5, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(110, 125, 110, 110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(43, 125, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(70, 115, 90, 90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-33, -34, -135, -135), 7000, 0.5, false, false, false, 2100);
        drivetrain.stop(400);
        drivetrain.boomerang(Pose(65, 65, 45, 45), 6000, 0.2, true, false, false);
        
        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(125, 75, 90, 90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(175, 75, 90, 90), 3000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(150, 75, -90, -90), 12000, 0.5, true, false, false);

        // Elim end
        drivetrain.boomerang(Pose(308, 5, 90, 90), 12000, 0.3, false, false, true, 2000);
        drivetrain.waitUntil(10);
        doinkers.setStates(false, true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(200);
        drivetrain.turnToHeading(-45, 1500, false, false);
    }
    //Pos
    void redPos_6_Qual()
    {
        startPose = Pose(-130, 40, 0, 0);
        drivetrain.runOdom(startPose);
        intake.setRingSortColour(true);
        intake.holdPosition(subsystems::DEFAULT);

        //Center rings
        drivetrain.boomerang(Pose(-158.5, 146, -40, -40), 8000, 0.3, false, false, false);
        doinkers.setStates(false, true);
        drivetrain.stop(200);
        drivetrain.swingToHeading(-24, true, false, false, false);
        doinkers.setStates(true, true);
        drivetrain.stop(200);

        //Getting mogo
        drivetrain.boomerang(Pose(-85, 85, 161, 161), 8000, 0.2, true, false, true);
        drivetrain.waitUntil(35);
        mogo.setState(true);
        drivetrain.waitUntil(43);
        doinkers.setStates(false, false);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        //Collecting rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(-64, 110, 10, 10), 12000, 0.2, false, false, false);
        drivetrain.boomerang(Pose(-120, 140, -70, -70), 12000, 0.1, false, false, false);
        drivetrain.boomerang(Pose(-165, 90, -100, -100), 12000, 0.1, false, false, false);
        drivetrain.boomerang(Pose(-150, 45, -170, -170), 12000, 0.1, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(-95, 15, 90, 90), 12000, 0.4, false, false, false);

        //Corner
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(10, -30, 90, 90), 8000, 0.01, false, false, false, 1000);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(drivetrain.pose.x - 8, drivetrain.pose.y + 8, -45, -45), 4000, 0.1, true, false, false, 600);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(10, -30, 135, 135), 8000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);

        //Clearing
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(-60, -15, -90, -90), 12000, 0.1, true, false, false, 800);
        drivetrain.turnToHeading(105, 300, false, false);

        doinkers.setStates(false, true);
        drivetrain.swingToHeading(120, true, false, false, false, 400);
        drivetrain.turnToHeading(-45, 800, false, false);
        doinkers.setStates(false, false);
        mogo.setState(false);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 30, drivetrain.pose.y - 30, 135, 135), 12000, 0.1, true, false, false, 500);
        intake.autonFunctions(-12000);

        //Ladder
        drivetrain.boomerang(Pose(drivetrain.pose.x - 25, drivetrain.pose.y + 35, -45, -45), 12000, 0.5, false, false, false, 800);
        intake.holdPosition(subsystems::WALL);
        drivetrain.boomerang(Pose(drivetrain.pose.x - 105, drivetrain.pose.y + 70, -45, -45), 12000, 0.5, false, false, false, 900);
    }

    void redPos_6_Elim()
    {
        // startPose = Pose(-130, 40, 0, 0);
        // drivetrain.runOdom(startPose);
        // intake.setRingSortColour(true);
        // intake.holdPosition(subsystems::DEFAULT);

        // //Center rings
        // drivetrain.boomerang(Pose(-158.5, 146, -40, -40), 8000, 0.3, false, false, false);
        // doinkers.setStates(false, true);
        // drivetrain.stop(200);
        // drivetrain.swingToHeading(-24, true, false, false, false);
        // doinkers.setStates(true, true);
        // drivetrain.stop(200);

        // //Getting mogo
        // drivetrain.boomerang(Pose(-85, 85, 161, 161), 8000, 0.2, true, false, true);
        // drivetrain.waitUntil(35);
        // mogo.setState(true);
        // drivetrain.waitUntil(46);
        // doinkers.setStates(false, false);
        // drivetrain.waitUntilEnd();
        // drivetrain.stop(100);

        // //Collecting rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(-64, 110, 10, 10), 12000, 0.2, false, false, false);
        // drivetrain.boomerang(Pose(-125, 140, -70, -70), 12000, 0.1, false, false, false);
        // drivetrain.boomerang(Pose(-160, 45, -170, -170), 12000, 0.1, false, false, false);
        // drivetrain.stop(300);
        // drivetrain.boomerang(Pose(-95, 7, 90, 90), 12000, 0.4, false, false, false);

        // //Corner
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(-5, -25, 90, 90), 8000, 0.01, false, false, false, 1000);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(drivetrain.pose.x - 8, drivetrain.pose.y + 8, -45, -45), 4000, 0.1, true, false, false, 600);
        // intake.setIntakeLiftState(true);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(-5, -25, 135, 135), 8000, 0.1, false, false, false, 700);
        // intake.setIntakeLiftState(false);

        // //Clearing
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(-65, -10, -90, -90), 12000, 0.1, true, false, false, 800);
        // drivetrain.turnToHeading(105, 300, false, false);

        // doinkers.setStates(false, true);
        // drivetrain.swingToHeading(120, true, false, false, false, 400);
        // drivetrain.turnToHeading(-45, 800, false, false);
        // doinkers.setStates(false, false);
        // mogo.setState(false);
        // drivetrain.boomerang(Pose(drivetrain.pose.x + 30, drivetrain.pose.y - 30, 135, 135), 12000, 0.1, true, false, false, 500);
        // intake.autonFunctions(-12000);

        // //Third
        // drivetrain.boomerang(Pose(drivetrain.pose.x - 25, drivetrain.pose.y + 35, -45, -45), 12000, 0.5, false, false, false, 400);
        // drivetrain.boomerang(Pose(drivetrain.pose.x, drivetrain.pose.y + 70, 0, 0), 12000, 0.5, true, false, false, 900);
        // drivetrain.stop(200);

        startPose = Pose(-130, 40, 0, 0);
        drivetrain.runOdom(startPose);
        intake.setRingSortColour(true);
        intake.holdPosition(subsystems::DEFAULT);

        //Center rings
        drivetrain.boomerang(Pose(-158.5, 146, -40, -40), 8000, 0.3, false, false, false);
        doinkers.setStates(false, true);
        drivetrain.stop(200);
        drivetrain.swingToHeading(-24, true, false, false, false);
        doinkers.setStates(true, true);
        drivetrain.stop(200);

        //Getting mogo
        drivetrain.boomerang(Pose(-85, 85, 161, 161), 8000, 0.2, true, false, true);
        drivetrain.waitUntil(35);
        mogo.setState(true);
        drivetrain.waitUntil(43);
        doinkers.setStates(false, false);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        //Collecting rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(-64, 110, 10, 10), 12000, 0.2, false, false, false);
        drivetrain.boomerang(Pose(-120, 140, -70, -70), 12000, 0.1, false, false, false);
        drivetrain.boomerang(Pose(-165, 90, -100, -100), 12000, 0.1, false, false, false);
        drivetrain.boomerang(Pose(-150, 45, -170, -170), 12000, 0.1, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(-95, 15, 90, 90), 12000, 0.4, false, false, false);

        //Corner
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(10, -30, 90, 90), 8000, 0.01, false, false, false, 1000);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(drivetrain.pose.x - 8, drivetrain.pose.y + 8, -45, -45), 4000, 0.1, true, false, false, 600);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(10, -30, 135, 135), 8000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);

        //Clearing
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(-60, -15, -90, -90), 12000, 0.1, true, false, false, 800);
        drivetrain.turnToHeading(105, 300, false, false);

        doinkers.setStates(false, true);
        drivetrain.swingToHeading(120, true, false, false, false, 400);
        drivetrain.turnToHeading(-45, 800, false, false);
        doinkers.setStates(false, false);
        mogo.setState(false);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 30, drivetrain.pose.y - 30, 135, 135), 12000, 0.1, true, false, false, 500);
        intake.autonFunctions(-12000);

        //Third
        drivetrain.boomerang(Pose(drivetrain.pose.x - 25, drivetrain.pose.y + 35, -45, -45), 12000, 0.5, false, false, false, 400);
        drivetrain.boomerang(Pose(drivetrain.pose.x, drivetrain.pose.y + 70, 0, 0), 12000, 0.5, true, false, false, 1000);
        drivetrain.stop(200);
    }

    void redPos_1_4_Qual()
    {
        // startPose = Pose(-150, 30, -121, -121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(true);

        // // Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // // Alliance stake ring
        // drivetrain.swingToHeading(-90, false, true, false, false);
        // drivetrain.stop(100);

        // intake.autonFunctions(10000);
        // intake.setIntakeLiftState(true);
        // intake.holdPosition(subsystems::DEFAULT);
        // drivetrain.boomerang(Pose(-175, 65, -90, -90), 4500, 0.5, false, false, false);
        // drivetrain.stop(100);
        // intake.waitForRing(1000);
        // intake.autonFunctions(-750);
        // intake.setIntakeLiftState(false);

        // // Mogo
        // drivetrain.boomerang(Pose(-116, 131, 20, 20), 7000, 0.4, true, false, true);
        // drivetrain.waitUntil(42);
        // intake.autonFunctions(0);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();
        // drivetrain.stop(100);

        // // Second mogo ring
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(-65, 110, 90, 90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // // Next to Corner ring
        // drivetrain.boomerang(Pose(-90, 110, -90, -90), 12000, 0.1, true, false, false);
        // drivetrain.boomerang(Pose(-60, 50, 135, 135), 8000, 0.1, false, false, false);
        // goalGrabber.setState(true);
        // drivetrain.stop(400);

        // // Corner
        // drivetrain.boomerang(Pose(-10, 0, 135, 135), 8000, 0.7, false, false, false, 1000);
        // drivetrain.stop(300);
        // drivetrain.boomerang(Pose(drivetrain.pose.x - 7, drivetrain.pose.y + 7, -45, -45), 4000, 0.1, true, false, false, 800);
        // intake.setIntakeLiftState(true);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(-5, -5, 135, 135), 8000, 0.1, false, false, false, 1200);
        // intake.setIntakeLiftState(false);

        // drivetrain.boomerang(Pose(-40, 30, -45, -45), 8000, 0.1, true, false, false);
        // doinkers.setStates(false, true);
        // goalGrabber.setState(false);
        // drivetrain.stop(100);

        // // Clear
        // drivetrain.swingToHeading(110, true, false, false, false, 200);
        // drivetrain.swingToHeading(90, false, false, false, false, 200);
        // drivetrain.turnToHeading(0, 1000, false, false);
        // doinkers.setStates(false, false);
        // drivetrain.stop(100);

        // //Drop mogo
        // drivetrain.boomerang(Pose(drivetrain.pose.x - 20, 70, 0, 0), 12000, 0.1, false, false, false, 1000);
        // mogo.setState(false);
        // intake.autonFunctions(-1000);
        // drivetrain.stop(500);

        // //Qual end, ladder
        // drivetrain.boomerang(Pose(-103, 108, -45, -45), 10000, 0.5, false, false, false, 2000);
        // intake.holdPosition(subsystems::ALLIANCE);
        // drivetrain.stop(100);

        startPose = Pose(-150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(true);

        // Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        // Alliance stake ring
        drivetrain.boomerang(Pose(-115, 70, 59, 59), 8000, 0.1, true, false, false);
        drivetrain.stop(100);

        intake.autonFunctions(10000);
        intake.setIntakeLiftState(true);
        intake.holdPosition(subsystems::DEFAULT);
        drivetrain.boomerang(Pose(-178, 70, -90, -90), 3000, 0.1, false, false, false);
        drivetrain.stop(100);
        intake.waitForRing(800);
        intake.autonFunctions(-750);
        intake.setIntakeLiftState(false);

        // Mogo
        drivetrain.boomerang(Pose(-116, 131, 20, 20), 7000, 0.4, true, false, true);
        drivetrain.waitUntil(40);
        intake.autonFunctions(0);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        // Second mogo ring
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(-40, 105, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);
                  
        // Next to Corner ring
        drivetrain.boomerang(Pose(-60, 110, -90, -90), 12000, 0.1, true, false, false);
        drivetrain.boomerang(Pose(-50, 50, 135, 135), 8000, 0.1, false, false, false);
        goalGrabber.setState(true);
        drivetrain.stop(400);

        // Corner
        drivetrain.boomerang(Pose(20, -10, 135, 135), 8000, 0.7, false, false, false, 800);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(drivetrain.pose.x - 7.5, drivetrain.pose.y + 7.5, -45, -45), 3000, 0.1, true, false, false, 650);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(15, -18, 135, 135), 8000, 0.1, false, false, false, 800);
        intake.setIntakeLiftState(false);

        drivetrain.boomerang(Pose(-42, 28, -45, -45), 8000, 0.1, true, false, false, 800);
        doinkers.setStates(false, true);
        goalGrabber.setState(false);
        drivetrain.stop(100);

        // Clear
        drivetrain.swingToHeading(110, true, false, false, false, 200);
        drivetrain.swingToHeading(90, false, false, false, false, 200);
        drivetrain.turnToHeading(0, 800, false, false);
        doinkers.setStates(false, false);
        drivetrain.stop(100);

        //Drop mogo
        drivetrain.boomerang(Pose(drivetrain.pose.x - 20, 70, 0, 0), 12000, 0.1, false, false, false, 1000);
        mogo.setState(false);
        intake.autonFunctions(-1000);
        drivetrain.stop(500);

        //Qual end, ladder
        drivetrain.boomerang(Pose(-123, 128, -45, -45), 10000, 0.5, false, false, true, 2000);
        drivetrain.waitUntil(20);
        intake.holdPosition(subsystems::ALLIANCE);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        
    }

    void redPos_1_4_Elim()
    {
        startPose = Pose(-150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(true);

        // Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        // Alliance stake ring
        drivetrain.boomerang(Pose(-115, 70, 59, 59), 8000, 0.1, true, false, false);
        drivetrain.stop(100);

        intake.autonFunctions(10000);
        intake.setIntakeLiftState(true);
        intake.holdPosition(subsystems::DEFAULT);
        drivetrain.boomerang(Pose(-178, 70, -90, -90), 3000, 0.1, false, false, false);
        drivetrain.stop(100);
        intake.waitForRing(800);
        intake.autonFunctions(-750);
        intake.setIntakeLiftState(false);

        // Mogo
        drivetrain.boomerang(Pose(-113, 131, 20, 20), 7000, 0.4, true, false, true);
        drivetrain.waitUntil(40);
        intake.autonFunctions(0);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        // Second mogo ring
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(-40, 105, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);
                  
        // Next to Corner ring
        drivetrain.boomerang(Pose(-60, 110, -90, -90), 12000, 0.1, true, false, false);
        drivetrain.boomerang(Pose(-50, 50, 135, 135), 8000, 0.1, false, false, false);
        goalGrabber.setState(true);
        drivetrain.stop(400);

        // Corner
        drivetrain.boomerang(Pose(20, -10, 135, 135), 8000, 0.7, false, false, false, 800);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(drivetrain.pose.x - 7.5, drivetrain.pose.y + 7.5, -45, -45), 3000, 0.1, true, false, false, 650);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(15, -18, 135, 135), 8000, 0.1, false, false, false, 800);
        intake.setIntakeLiftState(false);

        drivetrain.boomerang(Pose(-42, 28, -45, -45), 8000, 0.1, true, false, false, 800);
        doinkers.setStates(false, true);
        goalGrabber.setState(false);
        drivetrain.stop(100);

        // Clear
        drivetrain.swingToHeading(110, true, false, false, false, 200);
        drivetrain.swingToHeading(90, false, false, false, false, 200);
        drivetrain.turnToHeading(0, 800, false, false);
        doinkers.setStates(false, false);
        drivetrain.stop(100);

        //Drop mogo
        drivetrain.boomerang(Pose(drivetrain.pose.x - 20, 70, 0, 0), 12000, 0.1, false, false, false, 1000);
        mogo.setState(false);
        intake.autonFunctions(-1000);
        drivetrain.stop(500);

        //Elim end, third
        drivetrain.boomerang(Pose(-25, 100, 20, 20), 12000, 0.5, true, false, false, 2000);
        drivetrain.stop(100);
    }

    //Sawp
    void redSawp_1_4_1_Qual()
    {
        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(true);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(110, 40, -59, -59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        drivetrain.boomerang(Pose(125, 132, 20, 20), 7000, 0.4, true, false, true);
        drivetrain.waitUntil(41.8);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(93, 159.5, -45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(40, 161, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(105, 125, 110, 110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(43, 115, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Get alliance stake ring
        drivetrain.boomerang(Pose(70, 115, 90, 90), 10000, 0.1, true, false, false);
        drivetrain.boomerang(Pose(135, 70, 90, 90), 10000, 0.1, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(195, 70, 90, 90), 4000, 0.2, false, false, false);
        intake.setIntakeLiftState(false);
        pros::delay(200);
        drivetrain.boomerang(Pose(170, 70, -90, -90), 8000, 0.1, true, false, false);

        //Dropping mogo
        drivetrain.boomerang(Pose(200, 70, 90, 90), 12000, 0.5, false, false, false);
        mogo.setState(false);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(280, 70, 90, 90), 12000, 0.5, false, false, false);
        drivetrain.stop(100);

        //Getting second goal
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(253, 140, -25, -25), 7000, 0.5, true, false, true);
        drivetrain.waitUntil(29);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        //last ring
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(320, 128, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(800);

        //Ladder
        drivetrain.boomerang(Pose(230, 150, -90, -90), 12000, 0.1, true, false, false, 1500);
        drivetrain.stop(200);
    }

//Blue
    //Neg
    void blueNeg_6_1_Qual()
    {
        startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(false);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 50, 59, 59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DOUBLERING);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 120, 132, -30, -30), 6000, 0.4, true, false, true);
        drivetrain.waitUntil(44);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 165.5, 45), 8000, 0.6, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 32, 167.5, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 1500);
        drivetrain.stop(200);

        drivetrain.boomerang(Pose(drivetrain.pose.x - 10, drivetrain.pose.y + 10, -45, -45), 3000, 0.1, true, false, false, 600);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 600);
        intake.setIntakeLiftState(false);
        drivetrain.stop(300);

        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 80, -45, -45), 6000, 0.2, true, false, false);

        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 108, 80, -90, -90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 80, -90, -90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 80, 90, 90), 12000, 0.1, true, false, false);

        //Qual end
        drivetrain.boomerang(Pose(FIELD_WIDTH - 150, 115, 0, 0), 12000, 0.1, false, false, false, 800);
        intake.holdPosition(subsystems::ALLIANCE);
        drivetrain.stop(200);
    }

    void blueNeg_6_1_Elim()
    {
        // startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(false);

        // //Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 40, 59, 59), 12000, 0.4, true, false, false);
        // intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 125, 132, -20, -20), 6500, 0.4, true, false, true);
        // drivetrain.waitUntil(42);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 88, 165, 45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 37, 167, 90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 4, 135, 135), 6000, 0.1, false, false, false, 1500);
        // drivetrain.stop(200);

        // drivetrain.boomerang(Pose(drivetrain.pose.x - 8, drivetrain.pose.y + 8, -45, -45), 3000, 0.1, true, false, false, 800);
        // intake.setIntakeLiftState(true);
        // drivetrain.stop(300);
        // drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 4, 135, 135), 6000, 0.1, false, false, false, 800);
        // intake.setIntakeLiftState(false);
        // drivetrain.stop(300);

        // drivetrain.boomerang(Pose(FIELD_WIDTH - 65, 65, -45, -45), 6000, 0.2, true, false, false);

        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 83, -90, -90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 83, -90, -90), 4000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 83, 90, 90), 12000, 0.1, true, false, false);

        startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(false);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 50, 59, 59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DOUBLERING);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 120, 132, -30, -30), 6000, 0.4, true, false, true);
        drivetrain.waitUntil(44);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 165.5, 45), 8000, 0.6, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 32, 167.5, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 1500);
        drivetrain.stop(200);

        drivetrain.boomerang(Pose(drivetrain.pose.x - 10, drivetrain.pose.y + 10, -45, -45), 3000, 0.1, true, false, false, 600);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 600);
        intake.setIntakeLiftState(false);
        drivetrain.stop(300);

        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 80, -45, -45), 6000, 0.2, true, false, false);

        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 108, 80, -90, -90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 80, -90, -90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 80, 90, 90), 12000, 0.1, true, false, false);
        drivetrain.stop(100);
    }

    void blueNeg_5_1_Qual()
    {
        // startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(false);

        // //Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 40, 59, 59), 12000, 0.4, true, false, false);
        // intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 125, 132, -20, -20), 6500, 0.4, true, false, true);
        // drivetrain.waitUntil(42);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 88, 165, 45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 37, 167, 90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 4, 135, 135), 6000, 0.1, false, false, false, 1500);
        // drivetrain.stop(200);

        // drivetrain.boomerang(Pose(FIELD_WIDTH - 65, 65, -45, -45), 6000, 0.2, true, false, false);

        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 79, -90, -90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 79, -90, -90), 4000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 79, 90, 90), 12000, 0.1, true, false, false);

        // //Qual end
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 150, 110, 0, 0), 12000, 0.1, false, false, false, 800);
        // intake.holdPosition(subsystems::ALLIANCE);
        // drivetrain.stop(200);

        startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(false);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 50, 59, 59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DOUBLERING);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 122, 132, -30, -30), 6000, 0.4, true, false, true);
        drivetrain.waitUntil(41);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 165.5, 45), 8000, 0.6, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 32, 167.5, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 1500);
        drivetrain.stop(200);

        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 80, -45, -45), 6000, 0.2, true, false, false);

        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 108, 85, -90, -90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 85, -90, -90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 85, 90, 90), 12000, 0.1, true, false, false);

        //Qual end
        drivetrain.boomerang(Pose(FIELD_WIDTH - 150, 135, 0, 0), 12000, 0.1, false, false, false, 1000);
        intake.holdPosition(subsystems::ALLIANCE);
        drivetrain.stop(200);

    }

    void blueNeg_5_1_Elim()
    {
        // startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(false);

        // //Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // //Mogo
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 40, 59, 59), 12000, 0.4, true, false, false);
        // intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 125, 132, -20, -20), 6500, 0.4, true, false, true);
        // drivetrain.waitUntil(42);
        // mogo.setState(true);
        // drivetrain.waitUntilEnd();

        // //2 mid rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 88, 165, 45), 8000, 0.6, false, false, false);
        // drivetrain.stop(100);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 37, 167, 90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Going back, then 3rd ring
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // //Corner
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 4, 135, 135), 6000, 0.1, false, false, false, 1500);
        // drivetrain.stop(200);
        
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 65, 65, -45, -45), 6000, 0.2, true, false, false);

        // //Alliance stake ring
        // goalGrabber.setState(false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 79, -90, -90), 12000, 0.2, false, false, false);
        // intake.setIntakeLiftState(true);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 79, -90, -90), 4000, 0.5, false, false, false);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 79, 90, 90), 12000, 0.1, true, false, false);

        // // Elim end
        // drivetrain.boomerang(Pose(FIELD_WIDTH - 308, 0, -90, -90), 12000, 0.3, false, false, true, 2000);
        // drivetrain.waitUntil(10);
        // doinkers.setStates(true, false);
        // drivetrain.waitUntilEnd();
        // drivetrain.stop(200);
        // drivetrain.turnToHeading(45, 1500, false, false);

        startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(false);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 50, 59, 59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DOUBLERING);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 120, 132, -30, -30), 6000, 0.4, true, false, true);
        drivetrain.waitUntil(44);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 165.5, 45), 8000, 0.6, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 32, 167.5, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 1500);
        drivetrain.stop(500);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 80, -45, -45), 6000, 0.2, true, false, false);

        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 108, 80, -90, -90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 80, -90, -90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 80, 90, 90), 12000, 0.1, true, false, false);

        // Elim end
        drivetrain.boomerang(Pose(FIELD_WIDTH - 308, 0, -90, -90), 12000, 0.3, false, false, true, 2000);
        drivetrain.waitUntil(10);
        doinkers.setStates(true, false);
        drivetrain.waitUntilEnd();
        drivetrain.stop(200);
        drivetrain.turnToHeading(45, 1500, false, false);
    }

    void blueNeg_6_Qual()
    {
        startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setRingSortColour(false);

        //Mogo
        drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 50, 59, 59), 12000, 0.4, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 120, 132, -30, -30), 6000, 0.4, true, false, true);
        drivetrain.waitUntil(44);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 165.5, 45), 8000, 0.6, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 32, 167.5, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 1500);
        drivetrain.stop(500);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 80, -45, -45), 6000, 0.2, true, false, false);

        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 108, 80, -90, -90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 80, -90, -90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 80, 90, 90), 12000, 0.1, true, false, false);

        //Qual end
        drivetrain.boomerang(Pose(FIELD_WIDTH - 150, 110, 0, 0), 12000, 0.1, false, false, false, 800);
        intake.holdPosition(subsystems::ALLIANCE);
        drivetrain.stop(200);
    }

    void blueNeg_6_Elim()
    {
        startPose = Pose(FIELD_WIDTH - 150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setRingSortColour(false);

        //Mogo
        drivetrain.boomerang(Pose(FIELD_WIDTH - 100, 50, 59, 59), 12000, 0.4, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 120, 132, -30, -30), 6000, 0.4, true, false, true);
        drivetrain.waitUntil(44);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 165.5, 45), 8000, 0.6, false, false, false);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 32, 167.5, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(FIELD_WIDTH - 110, 140, -110, -110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 46, 132, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.boomerang(Pose(FIELD_WIDTH - 70, 115, -90, -90), 12000, 0.1, true, false, false);
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH + 10, 0, 135, 135), 6000, 0.1, false, false, false, 1500);
        drivetrain.stop(500);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 80, 80, -45, -45), 6000, 0.2, true, false, false);

        //Alliance stake ring
        goalGrabber.setState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 108, 80, -90, -90), 12000, 0.2, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 160, 80, -90, -90), 4000, 0.5, false, false, false);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(FIELD_WIDTH - 130, 80, 90, 90), 12000, 0.1, true, false, false);

        // Elim end
        drivetrain.boomerang(Pose(FIELD_WIDTH - 308, 0, -90, -90), 12000, 0.3, false, false, true, 2000);
        drivetrain.waitUntil(10);
        doinkers.setStates(true, false);
        drivetrain.waitUntilEnd();
        drivetrain.stop(200);
        drivetrain.turnToHeading(45, 1500, false, false);
    }
    //Pos
    void bluePos_6_Qual()
    {
        startPose = Pose(130, 40, 0, 0);
        drivetrain.runOdom(startPose);
        intake.setRingSortColour(false);
        intake.holdPosition(subsystems::DEFAULT);

        //Center rings
        drivetrain.boomerang(Pose(157, 149.5, 40, 40), 8000, 0.3, false, false, false, 2500);
        doinkers.setStates(true, false);
        drivetrain.stop(200);
        drivetrain.swingToHeading(27, false, false, false, false, 800);
        doinkers.setStates(true, true);
        drivetrain.stop(200);
        
        //Getting mogo
        drivetrain.boomerang(Pose(85, 85, -161, -161), 8000, 0.2, true, false, true, 2000);
        drivetrain.waitUntil(35);
        mogo.setState(true);
        drivetrain.waitUntil(41);
        doinkers.setStates(false, false);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        //Collecting rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(50, 110, -10, -10), 12000, 0.2, false, false, false, 1800);
        drivetrain.boomerang(Pose(100, 128, 70, 70), 12000, 0.1, false, false, false, 1800);
        drivetrain.boomerang(Pose(150, 100, 110, 110), 12000, 0.1, false, false, false, 1800);
        drivetrain.boomerang(Pose(155, 80, 170, 170), 12000, 0.1, false, false, false, 1800);
        drivetrain.stop(300);
        drivetrain.boomerang(Pose(90, 20, -90, -90), 12000, 0.4, false, false, false, 1800);

        //Corner
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-5, -20, -90, -90), 12000, 0.01, false, false, false, 800);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 8, drivetrain.pose.y + 8, 45, 45), 4000, 0.1, true, false, false, 700);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(-10, -30, -135, -135), 8000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);

        //Clearing
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(55, -10, 90, 90), 12000, 0.1, true, false, false, 800);
        drivetrain.turnToHeading(-115, 300, false, false);

        doinkers.setStates(true, false);
        drivetrain.stop(250);
        drivetrain.turnToHeading(45, 800, false, false);
        doinkers.setStates(false, false);
        mogo.setState(false);
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(drivetrain.pose.x - 30, drivetrain.pose.y - 30, -135, -135), 12000, 0.1, true, false, false, 500);
        
        //Ladder
        drivetrain.boomerang(Pose(drivetrain.pose.x + 25, drivetrain.pose.y + 35, 45, 45), 12000, 0.5, false, false, false, 500);
        goalGrabber.setState(false);
        intake.holdPosition(subsystems::DOUBLERING);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 130, drivetrain.pose.y + 115, 45, 45), 12000, 0.5, false, false, false, 900);
        drivetrain.stop(200);
    }

    void bluePos_6_Elim()
    {
        // startPose = Pose(130, 40, 0, 0);
        // drivetrain.runOdom(startPose);
        // intake.setRingSortColour(false);
        // intake.holdPosition(subsystems::DEFAULT);
    
        // //Center rings
        // drivetrain.boomerang(Pose(157, 149.5, 40, 40), 8000, 0.3, false, false, false);
        // doinkers.setStates(true, false);
        // drivetrain.stop(200);
        // drivetrain.swingToHeading(27, false, false, false, false, 800);
        // doinkers.setStates(true, true);
        // drivetrain.stop(200);
        
        // //Getting mogo
        // drivetrain.boomerang(Pose(85, 85, -161, -161), 8000, 0.2, true, false, true);
        // drivetrain.waitUntil(35);
        // mogo.setState(true);
        // drivetrain.waitUntil(42);
        // doinkers.setStates(false, false);
        // drivetrain.waitUntilEnd();
        // drivetrain.stop(100);

        // //Collecting rings
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(50, 110, -10, -10), 12000, 0.2, false, false, false);
        // drivetrain.boomerang(Pose(95, 128, 70, 70), 12000, 0.1, false, false, false);
        // drivetrain.boomerang(Pose(155, 80, 170, 170), 12000, 0.1, false, false, false);
        // drivetrain.stop(300);
        // drivetrain.boomerang(Pose(90, 20, -90, -90), 12000, 0.4, false, false, false);

        // //Corner
        // goalGrabber.setState(true);
        // drivetrain.boomerang(Pose(10, -20, -90, -90), 12000, 0.01, false, false, false, 1000);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(drivetrain.pose.x + 10, drivetrain.pose.y + 10, 45, 45), 4000, 0.1, true, false, false, 800);
        // intake.setIntakeLiftState(true);
        // drivetrain.stop(300);
        // drivetrain.boomerang(Pose(0, -30, -135, -135), 8000, 0.1, false, false, false, 700);
        // intake.setIntakeLiftState(false);

        // //Clearing
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(60, -10, 90, 90), 12000, 0.1, true, false, false, 500);
        // drivetrain.turnToHeading(-120, 350, false, false);

        // doinkers.setStates(true, false);
        // drivetrain.stop(250);
        // drivetrain.turnToHeading(45, 1000, false, false);
        // doinkers.setStates(false, false);
        // mogo.setState(false);
        // intake.autonFunctions(-12000);
        // drivetrain.boomerang(Pose(drivetrain.pose.x - 30, drivetrain.pose.y - 30, -135, -135), 12000, 0.1, true, false, false, 500);

        // //Third
        // drivetrain.boomerang(Pose(drivetrain.pose.x + 25, drivetrain.pose.y + 35, 45, 45), 12000, 0.5, false, false, false, 500);
        // drivetrain.boomerang(Pose(drivetrain.pose.x, drivetrain.pose.y + 70, 0, 0), 12000, 0.5, true, false, false, 600);
        // drivetrain.stop(200);

        startPose = Pose(130, 40, 0, 0);
        drivetrain.runOdom(startPose);
        intake.setRingSortColour(false);
        intake.holdPosition(subsystems::DEFAULT);

        //Center rings
        drivetrain.boomerang(Pose(157, 149.5, 40, 40), 8000, 0.3, false, false, false, 2500);
        doinkers.setStates(true, false);
        drivetrain.stop(200);
        drivetrain.swingToHeading(27, false, false, false, false, 800);
        doinkers.setStates(true, true);
        drivetrain.stop(200);
        
        //Getting mogo
        drivetrain.boomerang(Pose(85, 85, -161, -161), 8000, 0.2, true, false, true, 2000);
        drivetrain.waitUntil(35);
        mogo.setState(true);
        drivetrain.waitUntil(42);
        doinkers.setStates(false, false);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);


        //Collecting rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(50, 110, -10, -10), 12000, 0.2, false, false, false, 1800);
        drivetrain.boomerang(Pose(100, 128, 70, 70), 12000, 0.1, false, false, false, 1800);
        drivetrain.boomerang(Pose(150, 100, 110, 110), 12000, 0.1, false, false, false, 1800);
        drivetrain.boomerang(Pose(145, 75, 170, 170), 12000, 0.1, false, false, false, 1800);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(120, 20, -110, -110), 12000, 0.1, false, false, false, 1200);

        //Corner
        goalGrabber.setState(true);
        drivetrain.boomerang(Pose(-30, -20, -90, -90), 12000, 0.01, false, false, false, 1200);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 8.5, drivetrain.pose.y + 8.5, 45, 45), 4000, 0.1, true, false, false, 700);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(-35, -30, -135, -135), 8000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);

        //Clearing
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(35, -20, 90, 90), 12000, 0.1, true, false, false, 1000);
        drivetrain.turnToHeading(-115, 300, false, false);

        doinkers.setStates(true, false);
        drivetrain.stop(250);
        drivetrain.turnToHeading(45, 800, false, false);
        doinkers.setStates(false, false);
        mogo.setState(false);
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(drivetrain.pose.x - 30, drivetrain.pose.y - 30, -135, -135), 12000, 0.1, true, false, false, 500);
        
        //Third
        drivetrain.boomerang(Pose(drivetrain.pose.x + 25, drivetrain.pose.y + 35, 45, 45), 12000, 0.5, false, false, false, 500);
        drivetrain.boomerang(Pose(drivetrain.pose.x, drivetrain.pose.y + 120, 0, 0), 12000, 0.5, true, false, false, 800);
        drivetrain.stop(200);
    }

    void bluePos_1_4_Qual()
    {
        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(false);

        // Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        // Alliance stake ring
        drivetrain.boomerang(Pose(120, 65, -59, -59), 8000, 0.1, true, false, false);
        drivetrain.stop(100);

        intake.autonFunctions(12000);
        intake.setIntakeLiftState(true);
        intake.holdPosition(subsystems::DEFAULT);
        drivetrain.boomerang(Pose(174, 65, 90, 90), 3500, 0.1, false, false, false);
        drivetrain.stop(50);
        intake.waitForRing(800);
        intake.autonFunctions(0);
        intake.setIntakeLiftState(false);

        // Mogo
        drivetrain.boomerang(Pose(117, 131, -20, -20), 6500, 0.3, true, false, true);
        drivetrain.waitUntil(5);
            intake.autonFunctions(0);
        drivetrain.waitUntil(42);
            mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        // Second mogo ring
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(65, 110, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        // Next to Corner ring
        drivetrain.boomerang(Pose(90, 110, 90, 90), 12000, 0.1, true, false, false);
        drivetrain.boomerang(Pose(60, 50, -135, -135), 8000, 0.1, false, false, false);
        goalGrabber.setState(true);
        drivetrain.stop(400);

        // Corner
        drivetrain.boomerang(Pose(0, -5, -135, -135), 8000, 0.7, false, false, false, 1000);
        drivetrain.stop(500);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 8, drivetrain.pose.y + 8, 45, 45), 4000, 0.1, true, false, false, 800);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(-10, -15, -135, -135), 6000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(40, 30, 45, 45), 8000, 0.1, true, false, false, 1000);
        doinkers.setStates(true, false);
        goalGrabber.setState(false);
        drivetrain.stop(100);

        // Clear
        drivetrain.turnToHeading(-140, 300, false, false);
        drivetrain.turnToHeading(0, 1000, false, false);
        doinkers.setStates(false, false);
        drivetrain.stop(100);

        //Drop mogo
        drivetrain.boomerang(Pose(drivetrain.pose.x + 20, 60, 0, 0), 12000, 0.1, false, false, false, 800);
        mogo.setState(false);
        intake.autonFunctions(-1000);
        drivetrain.stop(500);

        //Qual end, ladder
        drivetrain.boomerang(Pose(120, 110, 45, 45), 10000, 0.5, false, false, false, 2000);
        intake.holdPosition(subsystems::ALLIANCE);
        drivetrain.stop(100);
    }

    void bluePos_1_4_Elim()
    {
        // startPose = Pose(150, 30, 121, 121);

        // drivetrain.runOdom(startPose);
        // intake.setLoadStartingPosition();
        // intake.setRingSortColour(false);

        // // Alliance Stake
        // intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        // pros::delay(450);

        // // Alliance stake ring
        // drivetrain.swingToHeading(90, true, true, false, false);
        // drivetrain.stop(100);

        // intake.autonFunctions(12000);
        // intake.setIntakeLiftState(true);
        // intake.holdPosition(subsystems::DEFAULT);
        // drivetrain.boomerang(Pose(165, 68, 90, 90), 4500, 0.5, false, false, false);
        // drivetrain.stop(50);
        // intake.waitForRing(1000);
        // intake.autonFunctions(-1000);
        // intake.setIntakeLiftState(false);

        // // Mogo
        // drivetrain.boomerang(Pose(117, 131, -20, -20), 6500, 0.3, true, false, true);
        // drivetrain.waitUntil(5);
        //     intake.autonFunctions(0);
        // drivetrain.waitUntil(43);
        //     mogo.setState(true);
        // drivetrain.waitUntilEnd();
        // drivetrain.stop(100);

        // // Second mogo ring
        // intake.autonFunctions(12000);
        // drivetrain.boomerang(Pose(65, 110, -90, -90), 12000, 0.1, false, false, false);
        // drivetrain.stop(100);

        // // Next to Corner ring
        // drivetrain.boomerang(Pose(90, 110, 90, 90), 12000, 0.1, true, false, false);
        // drivetrain.boomerang(Pose(60, 50, -135, -135), 8000, 0.1, false, false, false);
        // goalGrabber.setState(true);
        // drivetrain.stop(400);

        // // Corner
        // drivetrain.boomerang(Pose(10, 10, -135, -135), 8000, 0.7, false, false, false, 1000);
        // drivetrain.stop(300);
        // drivetrain.boomerang(Pose(drivetrain.pose.x + 7, drivetrain.pose.y + 7, 45, 45), 4000, 0.1, true, false, false, 800);
        // intake.setIntakeLiftState(true);
        // drivetrain.stop(200);
        // drivetrain.boomerang(Pose(0, 0, -135, -135), 6000, 0.1, false, false, false, 700);
        // intake.setIntakeLiftState(false);
        // drivetrain.boomerang(Pose(40, 30, 45, 45), 8000, 0.1, true, false, false, 1000);
        // doinkers.setStates(true, false);
        // goalGrabber.setState(false);
        // drivetrain.stop(100);

        // // Clear
        // drivetrain.swingToHeading(-110, false, false, false, false, 800);
        // drivetrain.turnToHeading(0, 1000, false, false);
        // doinkers.setStates(false, false);
        // drivetrain.stop(100);

        // //Drop mogo
        // drivetrain.boomerang(Pose(drivetrain.pose.x + 20, 70, 0, 0), 12000, 0.1, false, false, false, 1000);
        // mogo.setState(false);
        // intake.autonFunctions(-1000);
        // drivetrain.stop(500);

        // // Elim end, third
        // drivetrain.boomerang(Pose(35, 110, -20, -20), 12000, 0.5, true, false, false, 2000);
        // drivetrain.stop(100);

        startPose = Pose(150, 30, 121, 121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(false);

        // Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        // Alliance stake ring
        drivetrain.boomerang(Pose(120, 65, -59, -59), 8000, 0.1, true, false, false);
        drivetrain.stop(100);

        intake.autonFunctions(12000);
        intake.setIntakeLiftState(true);
        intake.holdPosition(subsystems::DEFAULT);
        drivetrain.boomerang(Pose(174, 65, 90, 90), 3500, 0.1, false, false, false);
        drivetrain.stop(50);
        intake.waitForRing(800);
        intake.autonFunctions(0);
        intake.setIntakeLiftState(false);

        // Mogo
        drivetrain.boomerang(Pose(117, 131, -20, -20), 6500, 0.3, true, false, true);
        drivetrain.waitUntil(5);
            intake.autonFunctions(0);
        drivetrain.waitUntil(42);
            mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        // Second mogo ring
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(65, 110, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        // Next to Corner ring
        drivetrain.boomerang(Pose(90, 110, 90, 90), 12000, 0.1, true, false, false);
        drivetrain.boomerang(Pose(60, 50, -135, -135), 8000, 0.1, false, false, false);
        goalGrabber.setState(true);
        drivetrain.stop(400);

        // Corner
        drivetrain.boomerang(Pose(0, -5, -135, -135), 8000, 0.7, false, false, false, 1000);
        drivetrain.stop(500);
        drivetrain.boomerang(Pose(drivetrain.pose.x + 8, drivetrain.pose.y + 8, 45, 45), 4000, 0.1, true, false, false, 800);
        intake.setIntakeLiftState(true);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(-10, -15, -135, -135), 6000, 0.1, false, false, false, 700);
        intake.setIntakeLiftState(false);
        drivetrain.boomerang(Pose(40, 30, 45, 45), 8000, 0.1, true, false, false, 1000);
        doinkers.setStates(true, false);
        goalGrabber.setState(false);
        drivetrain.stop(100);

        // Clear
        drivetrain.turnToHeading(-140, 300, false, false);
        drivetrain.turnToHeading(0, 1000, false, false);
        doinkers.setStates(false, false);
        drivetrain.stop(100);

        //Drop mogo
        drivetrain.boomerang(Pose(drivetrain.pose.x + 20, 60, 0, 0), 12000, 0.1, false, false, false, 800);
        mogo.setState(false);
        intake.autonFunctions(-1000);
        drivetrain.stop(500);

        // Elim end, third
        drivetrain.boomerang(Pose(35, 100, -20, -20), 12000, 0.5, true, false, false, 1500);
        drivetrain.stop(100);
    }

    //Sawp
    void blueSawp_1_4_1_Qual()
    {
        startPose = Pose(-150, 30, -121, -121);

        drivetrain.runOdom(startPose);
        intake.setLoadStartingPosition();
        intake.setRingSortColour(false);

        //Alliance Stake
        intake.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        pros::delay(450);

        //Mogo
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(-110, 40, 59, 59), 12000, 0.4, true, false, false);
        intake.holdPosition(subsystems::LiftPosition::DEFAULT);
        drivetrain.boomerang(Pose(-125, 132, -20, -20), 7000, 0.4, true, false, true);
        drivetrain.waitUntil(42);
        mogo.setState(true);
        drivetrain.waitUntilEnd();

        //2 mid rings
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(-93, 164, 45), 8000, 0.6, false, false, false);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(-40, 167, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Going back, then 3rd ring
        drivetrain.boomerang(Pose(-110, 155, -110, -110), 12000, 0.2, true, false, false);
        drivetrain.boomerang(Pose(-43, 135, 90, 90), 12000, 0.1, false, false, false);
        drivetrain.stop(100);

        //Get alliance stake ring
        drivetrain.boomerang(Pose(-70, 135, -90, -90), 10000, 0.1, true, false, false);
        drivetrain.boomerang(Pose(-120, 90, -90, -90), 10000, 0.1, false, false, false);
        intake.setIntakeLiftState(true);
        drivetrain.stop(100);
        drivetrain.boomerang(Pose(-195, 90, -90, -90), 4000, 0.2, false, false, false);
        intake.setIntakeLiftState(false);
        pros::delay(200);
        drivetrain.boomerang(Pose(-170, 90, 90, 90), 8000, 0.1, true, false, false);

        //Dropping mogo
        drivetrain.boomerang(Pose(-200, 90, -90, -90), 12000, 0.5, false, false, false);
        mogo.setState(false);
        drivetrain.stop(200);
        drivetrain.boomerang(Pose(-250, 90, -90, -90), 12000, 0.5, false, false, false);
        drivetrain.stop(100);

        //Getting second goal
        intake.autonFunctions(-12000);
        drivetrain.boomerang(Pose(-218, 160, 25, 25), 7000, 0.5, true, false, true);
        drivetrain.waitUntil(29);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);

        //last ring
        intake.autonFunctions(12000);
        drivetrain.boomerang(Pose(-305, 148, -90, -90), 12000, 0.1, false, false, false);
        drivetrain.stop(800);

        //Ladder
        drivetrain.boomerang(Pose(-205, 165, 90, 90), 12000, 0.1, true, false, false, 1500);
        drivetrain.stop(200);
    }


void autonomous() 
{
    // bluePos_6_Elim();
    selector.run_auton();
//Blue side goal rush
/*
    drivetrain.runOdom(Pose(0, 0, 0));
    intake.setRingSortColour(false);
    lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

    //Rush
    drivetrain.boomerang(Pose(5, 83, 10), 8000, 0.5, false, false, true);
    drivetrain.waitUntil(20);
    intake.autonFunctions(12000);
    drivetrain.waitUntil(80);
    doinker.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Pullback
    drivetrain.boomerang(Pose(0, 65, -170), 6000, 0.5, true, false, false);
    doinker.setState(false);
    drivetrain.stop(100);
    pros::delay(300);

    //Align
    drivetrain.turnToHeading(24, 600, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(10, 75, 24), 3000, 0.5, false, false, false);
    drivetrain.stop(100);

    drivetrain.drive(-11, 700, 12000, false);
    drivetrain.turnToHeading(24, 400, false, false);
    intake.autonFunctions(-12000);
    drivetrain.stop(100);

    //Score
    lift.holdPosition(subsystems::LiftPosition::TIP);
    pros::delay(400);


    //Pulloff
    drivetrain.drive(15, 700, 12000, true);
    pros::delay(200);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    drivetrain.drive(-20, 600, 12000, false);
    drivetrain.stop(100);

    //Mogo
    drivetrain.turnToHeading(-90, 600, false, false);

    drivetrain.boomerang(Pose(90, 75, 80), 3000, 0.5, true, false, false);
    mogo.setState(true);
    drivetrain.stop(100);

    //Ring
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(50, 85, -60), 6000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.drive(-10, 400, 12000, false);

    //Corner ring
    drivetrain.boomerang(Pose(-10, -12, -170), 6000, 0.1, false, false, false, 2000);

    drivetrain.drive(-30, 600, 12000, false);
    

    mogo.setState(false);
    drivetrain.drive(-30, 700, 12000, false);
    drivetrain.drive(30, 400, 12000, false);

    drivetrain.boomerang(Pose(0, 110, 35), 3000, 0.5, true, false, false);
    drivetrain.stop(100);

*/
    
//Red side goal rush
/*
    drivetrain.runOdom(Pose(0, 0, 0));
    intake.setRingSortColour(true);
    lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

    //Rush
    drivetrain.boomerang(Pose(-5, 83, -10), 8000, 0.5, false, false, true);
    drivetrain.waitUntil(20);
    intake.autonFunctions(12000);
    drivetrain.waitUntil(80);
    rushMech.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Pullback
    drivetrain.boomerang(Pose(0, 65, 170), 6000, 0.5, true, false, false);
    rushMech.setState(false);
    drivetrain.stop(100);
    pros::delay(300);

    //Align
    drivetrain.turnToHeading(-24, 600, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(-10, 75, -24), 3000, 0.5, false, false, false);
    drivetrain.stop(100);

    drivetrain.drive(-11, 700, 12000, false);
    drivetrain.turnToHeading(-24, 400, false, false);
    intake.autonFunctions(-12000);
    drivetrain.stop(100);

    //Score
    lift.holdPosition(subsystems::LiftPosition::TIP);
    pros::delay(400);


    //Pulloff
    drivetrain.drive(15, 700, 12000, true);
    pros::delay(200);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    drivetrain.drive(-20, 600, 12000, false);
    drivetrain.stop(100);

    //Mogo
    drivetrain.turnToHeading(90, 600, false, false);

    drivetrain.boomerang(Pose(-90, 75, -80), 3000, 0.5, true, false, false);
    mogo.setState(true);
    drivetrain.stop(100);

    //Ring
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(-50, 85, 60), 6000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.drive(-10, 400, 12000, false);

    //Corner ring
    drivetrain.boomerang(Pose(10, -12, 170), 6000, 0.1, false, false, false, 2000);

    drivetrain.drive(-30, 600, 12000, false);
    

    mogo.setState(false);
    drivetrain.drive(-30, 700, 12000, false);
    drivetrain.drive(30, 400, 12000, false);

    drivetrain.boomerang(Pose(0, 110, -35), 3000, 0.5, true, false, false);
    drivetrain.stop(100);
*/


//blue side alliance + 4 + ladder
/*  
        drivetrain.runOdom(Pose(0, 0, -90));
        intake.setRingSortColour(false);
        lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

        //Alliance stake
        drivetrain.turnToHeading(-123, 700, false, false);
        intake.autonFunctions(12000);
        drivetrain.drive(11, 400, 12000, false);
        intake.autonFunctions(-12000);
        lift.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        drivetrain.stop(400);
        drivetrain.drive(-20, 400, 12000, false);
        lift.holdPosition(subsystems::LiftPosition::DOUBLERING);

        //Getting mogo
        drivetrain.boomerang(Pose(9, 77, 20), 8000, 0.5, true, false, true);
        drivetrain.waitUntil(70);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(400);
        intake.autonFunctions(12000);

        //Getting 2 mid rings
        drivetrain.turnToHeading(0, 1000, false, false);
        drivetrain.boomerang(Pose(100, 127, 90), 3000, 0.65, false, false, false);

        //backing up
        drivetrain.boomerang(Pose(40, 80, -90), 6000, 0.3, true, false, false);
        drivetrain.stop(100);

        //Mid ring
        drivetrain.boomerang(Pose(70, 80, 90), 6000, false, false, false);
        drivetrain.stop(100);

        //Corner
        drivetrain.drive(-20, 600, 12000, false);
        drivetrain.stop(100);
        drivetrain.turnToHeading(180, 600, false, false);
        drivetrain.boomerang(Pose(113, -4, 135), 6000, 0.5, false, false, false);
        drivetrain.stop(100);

        //ladder
        drivetrain.drive(-20, 800, 12000, false);
        drivetrain.turnToHeading(-90, 600, false, false);
        drivetrain.boomerang(Pose(0, 90, 45), 6000, 0.75, false, false, true);
        pros::delay(500);
        lift.holdPosition(subsystems::LiftPosition::WALL);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);
*/

//red side alliance + 4 + ladder
/*
        drivetrain.runOdom(Pose(0, 0, 90));
        intake.setRingSortColour(true);
        lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

        //Alliance stake
        drivetrain.turnToHeading(123, 700, false, false);
        intake.autonFunctions(12000);
        drivetrain.drive(11, 400, 12000, false);
        intake.autonFunctions(-12000);
        lift.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        drivetrain.stop(400);
        drivetrain.drive(-20, 400, 12000, false);
        lift.holdPosition(subsystems::LiftPosition::DOUBLERING);

        //Getting mogo
        drivetrain.boomerang(Pose(-9, 77, -20), 8000, 0.5, true, false, true);
        drivetrain.waitUntil(70);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(400);
        intake.autonFunctions(12000);

        //Getting 2 mid rings
        drivetrain.turnToHeading(0, 1000, false, false);
        drivetrain.boomerang(Pose(-100, 124, -90), 3000, 0.65, false, false, false);

        //backing up
        drivetrain.boomerang(Pose(-40, 85, 90), 6000, 0.3, true, false, false);
        drivetrain.stop(100);

        //Mid ring
        drivetrain.boomerang(Pose(-75, 80, -90), 6000, false, false, false);
        drivetrain.stop(100);
        
        //Corner
        drivetrain.drive(-20, 600, 12000, false);
        drivetrain.stop(100);
        drivetrain.turnToHeading(-180, 600, false, false);
        drivetrain.boomerang(Pose(-108, -2, -135), 6000, 0.5, false, false, false);
        drivetrain.stop(100);

        //ladder
        drivetrain.drive(-20, 800, 12000, false);
        drivetrain.turnToHeading(90, 600, false, false);
        drivetrain.boomerang(Pose(-20, 100, 45), 6000, 0.75, false, false, true);
        pros::delay(500);
        lift.holdPosition(subsystems::LiftPosition::WALL);
        drivetrain.waitUntilEnd();
        drivetrain.stop(100);
*/

/*
    //ring rush 6 ring - blue
        drivetrain.runOdom(Pose (0, 0, 0));
        lift.holdPosition(subsystems::LiftPosition::DEFAULT);
        intake.setRingSortColour(false);

        //Rush
        drivetrain.boomerang(Pose(13, 96, 25), 6000, 0.35, false, false, true);
        drivetrain.waitUntil(50);
        doinker.setState(true);
        intake.autonFunctions(12000);
        drivetrain.waitUntilEnd();
        drivetrain.stop(300);
        intake.autonFunctions(3000);

        //Moving to mogo
        drivetrain.swingToHeading(50, false, true, false, false);
        drivetrain.drive(-75, 1000, 12000, true);
        drivetrain.waitUntil(65);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        intake.autonFunctions(12000);
        doinker.setState(false);

        //Picking up rings
        drivetrain.drive(20, 500, 12000, false);
        drivetrain.turnToHeading(90, 500, false, false);
        drivetrain.drive(85, 3000, 6000, false);

        //Preload
        drivetrain.swingToHeading(170, false, true, false, false);
        drivetrain.boomerang(Pose(32, 35, 180), 6000, 0.2, false, false, false);
        drivetrain.turnToHeading(180, 200, false, false);
        drivetrain.turnToHeading(133, 1200, false ,false);
        intake.autonFunctions(0);
        
        //Bottom ring from corner
        drivetrain.drive(55, 500, 12000, false);
        intake.autonFunctions(12000);
        drivetrain.drive(80, 600, 12000, false);
        drivetrain.drive(-33, 600, 12000, false);

        drivetrain.stop(300);
*/
/*
    //ring rush 6 ring - red side
        drivetrain.runOdom(Pose (0, 0, 0));
        lift.holdPosition(subsystems::LiftPosition::DEFAULT);
        intake.setRingSortColour(true);

        //Rush
        drivetrain.boomerang(Pose(-13, 96, -25), 6000, 0.35, false, false, true);
        drivetrain.waitUntil(50);
        rushMech.setState(true);
        intake.autonFunctions(12000);
        drivetrain.waitUntilEnd();
        drivetrain.stop(300);
        intake.autonFunctions(3000);

        //Moving to mogo
        drivetrain.swingToHeading(-50, true, true, false, false);
        drivetrain.drive(-80, 1000, 12000, true);
        drivetrain.waitUntil(65);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        intake.autonFunctions(12000);
        rushMech.setState(false);

        //Picking up rings
        drivetrain.drive(20, 500, 12000, false);
        drivetrain.turnToHeading(-90, 500, false, false);
        drivetrain.drive(85, 2500, 7000, false);

        //Preload
        drivetrain.swingToHeading(-170, true, true, false, false);
        drivetrain.boomerang(Pose(-34, 35, -180), 6000, 0.2, false, false, false);
        drivetrain.turnToHeading(-180, 200, false, false);
        drivetrain.turnToHeading(-130, 800, false ,false);
        intake.autonFunctions(0);
        
        //Bottom ring from corner
        drivetrain.drive(55, 500, 12000, false);
        intake.autonFunctions(12000);
        drivetrain.drive(80, 600, 12000, false);
        drivetrain.drive(-33, 600, 12000, false);

        //Top ring from 2 stack
        // drivetrain.boomerang(Pose(27, 7, 90), 10000, 0.5, false, false, false);
        // drivetrain.turnToHeading(90, 400, false, false);
        // rushMech.setState(true);
        // drivetrain.drive(-25, 500, 12000, false);
        // rushMech.setState(false);
        // pros::delay(100);
        // drivetrain.swingToHeading(65, false, false, false, false);
        // drivetrain.drive(50, 900, 12000, false);

        // drivetrain.boomerang(Pose(30, 26, 90), 6000, 0.5, false, false, false);
        // drivetrain.turnToHeading(90, 500, false, false);
        // drivetrain.drive(90, 1000, 6000, false);

        drivetrain.stop(300);
*/


    //Safe solo awp - blue side
/*

    drivetrain.runOdom(Pose(0, 0, -90));
    intake.setRingSortColour(false);
    lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

    //Alliance stake
    drivetrain.turnToHeading(-123, 700, false, false);
    intake.autonFunctions(12000);
    drivetrain.drive(11, 400, 12000, false);
    intake.autonFunctions(-12000);
    lift.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
    drivetrain.stop(400);
    drivetrain.drive(-20, 400, 12000, false);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    //Getting mogo
    drivetrain.boomerang(Pose(9, 77, 20), 8000, 0.5, true, false, true);
    drivetrain.waitUntil(70);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.stop(400);
    intake.autonFunctions(12000);
    //Getting 2 mid rings
    drivetrain.turnToHeading(0, 1000, false, false);
    drivetrain.boomerang(Pose(100, 129, 90), 3000, 0.65, false, false, false);



    ///////////////
/*
        /*drivetrain.runOdom(Pose(0, 0, -90));
        intake.setRingSortColour(false);
        lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

        //Alliance stake
        drivetrain.turnToHeading(-123, 700, false, false);
        intake.autonFunctions(12000);
        drivetrain.drive(11, 400, 12000, false);
        intake.autonFunctions(-12000);
        lift.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        drivetrain.stop(400);
        drivetrain.drive(-20, 400, 12000, false);
        lift.holdPosition(subsystems::LiftPosition::DOUBLERING);

        //Getting mogo
        drivetrain.boomerang(Pose(9, 77, 20), 8000, 0.5, true, false, true);
        drivetrain.waitUntil(70);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(400);
        intake.autonFunctions(12000);

        //Getting 2 mid rings
        drivetrain.turnToHeading(0, 1000, false, false);
        drivetrain.boomerang(Pose(100, 127, 90), 3000, 0.65, false, false, false);



///////////////////

        //Getting top stack ring
        drivetrain.boomerang(Pose(10, 10, 180), 8000, 0.6, true, false, false);
        intake.autonFunctions(-12000);

        //Pushing teammate off line
        drivetrain.boomerang(Pose(-60, 4, -60), 8000, 0.4, false, false, true);
        drivetrain.waitUntil(115);
        intake.autonFunctions(-12000);
        drivetrain.waitUntilEnd();
        drivetrain.stop(200);

        //Dropping mogo
        mogo.setState(false);
        drivetrain.drive(-30, 400, 12000, false);

        //Getting other mogo
        drivetrain.boomerang(Pose(-72, 72, -20), 8000, 0.7, true, false, false);
        mogo.setState(true);

        //Getting other ring
        drivetrain.boomerang(Pose(-135, 74, -100), 8000, 0.5, false, false, true);
        drivetrain.waitUntil(40);
        intake.autonFunctions(12000);
        drivetrain.waitUntilEnd();
        drivetrain.drive(-20, 500, 12000, false);

        //Touching ladder
        drivetrain.turnToHeading(95, 500, false, false);
        lift.holdPosition(subsystems::LiftPosition::LOAD);
        drivetrain.boomerang(Pose(-30, 95, 90), 8000, 0.1, false, false, false);
        drivetrain.stop(300);
*/


/*
    //safe solo awp - red side
        drivetrain.runOdom(Pose(0, 0, 90));
        intake.setRingSortColour(true);
        lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

        //Alliance stake
        drivetrain.turnToHeading(122, 700, false, false);
        intake.autonFunctions(12000);
        drivetrain.drive(11, 400, 12000, false);
        intake.autonFunctions(-12000);
        lift.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        drivetrain.stop(400);
        drivetrain.drive(-20, 400, 12000, false);
        lift.holdPosition(subsystems::LiftPosition::DOUBLERING);

        //Getting mogo
        drivetrain.boomerang(Pose(-9, 77, -20), 8000, 0.5, true, false, true);
        drivetrain.waitUntil(70);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(400);
        intake.autonFunctions(12000);

        //Getting 2 mid rings
        drivetrain.turnToHeading(0, 1000, false, false);
        drivetrain.boomerang(Pose(-100, 123, -90), 3000, 0.65, false, false, false);

        //Getting top stack ring
        drivetrain.boomerang(Pose(-10, 10, -180), 8000, 0.6, true, false, false);
        intake.autonFunctions(-12000);

        //Pushing teammate off line
        drivetrain.boomerang(Pose(60, 4, 60), 8000, 0.4, false, false, true);
        drivetrain.waitUntil(115);
        intake.autonFunctions(-12000);
        drivetrain.waitUntilEnd();
        drivetrain.stop(200);

        //Dropping mogo
        mogo.setState(false);
        drivetrain.drive(-30, 400, 12000, false);

        //Getting other mogo
        drivetrain.boomerang(Pose(75, 67, 20), 8000, 0.7, true, false, false);
        mogo.setState(true);

        //Getting other ring
        drivetrain.boomerang(Pose(135, 76, 100), 8000, 0.5, false, false, true);
        drivetrain.waitUntil(20);
        intake.autonFunctions(12000);
        drivetrain.waitUntilEnd();
        drivetrain.drive(-20, 500, 12000, false);

        //Touching ladder
        drivetrain.turnToHeading(-95, 500, false, false);
        lift.holdPosition(subsystems::LiftPosition::LOAD);
        drivetrain.boomerang(Pose(30, 88, -90), 8000, 0.1, false, false, false);
        drivetrain.stop(300);
*/

//2 ring red
/*
        drivetrain.runOdom(Pose(0, 0, -90));
        intake.setRingSortColour(true);
        lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

        //Alliance stake
        drivetrain.turnToHeading(-122.8, 400, false, false);
        intake.autonFunctions(12000);
        drivetrain.drive(11, 400, 12000, false);
        intake.autonFunctions(-12000);
        lift.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        drivetrain.stop(400);
        drivetrain.drive(-20, 400, 12000, false);
        lift.holdPosition(subsystems::LiftPosition::DOUBLERING);

        //Getting mogo
        drivetrain.boomerang(Pose(10, 77, 15), 8000, 0.5, true, false, true);
        drivetrain.waitUntil(70);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(400);
        intake.autonFunctions(12000);


        //Getting 2nd ring
        drivetrain.boomerang(Pose(80, 85, 90), 6000, 0.5, false, false, false);
        drivetrain.stop(600);
*/
//2 ring blue#

/*
        drivetrain.runOdom(Pose(0, 0, 90));
        intake.setRingSortColour(false);
        lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

        //Alliance stake
        drivetrain.turnToHeading(122.8, 400, false, false);
        intake.autonFunctions(12000);
        drivetrain.drive(11, 400, 12000, false);
        intake.autonFunctions(-12000);
        lift.holdPosition(subsystems::LiftPosition::AUTOALLIANCESLAM);
        drivetrain.stop(400);
        drivetrain.drive(-20, 400, 12000, false);
        lift.holdPosition(subsystems::LiftPosition::DOUBLERING);

        //Getting mogo
        drivetrain.boomerang(Pose(-10, 77, -15), 8000, 0.5, true, false, true);
        drivetrain.waitUntil(70);
        mogo.setState(true);
        drivetrain.waitUntilEnd();
        drivetrain.stop(400);
        intake.autonFunctions(12000);


        //Getting 2nd ring
        drivetrain.boomerang(Pose(-80, 85, -90), 6000, 0.5, false, false, false);
        drivetrain.stop(600);
*/

//x team 4 ring no allaince red
/*
    drivetrain.runOdom(Pose(0, 0, -90));
    intake.setRingSortColour(true);
    lift.holdPosition(subsystems::LiftPosition::DEFAULT);

    //Getting mogo
    drivetrain.boomerang(Pose(10, 77, 15), 8000, 0.5, true, false, true);
    drivetrain.waitUntil(72);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.stop(400);
    intake.autonFunctions(12000);

    //Getting 2nd ring
    drivetrain.boomerang(Pose(80, 75, 90), 6000, 0.5, false, false, false);
    drivetrain.stop(600);

    //Top ring from 2 stack
    drivetrain.boomerang(Pose(30, 16, -90), 6000, 0.5, false, false, false);
    drivetrain.turnToHeading(-90, 500, false, false);
    drivetrain.drive(90, 1000, 6000, false);
    drivetrain.stop(300);
    drivetrain.boomerang(Pose(120, 40, 90), 6000, 0.5, true, false, false);
    drivetrain.stop(300);
*/

//Skills
/*
    drivetrain.runOdom(Pose(0, 0, 0, 0));
    lift.holdPosition(subsystems::LiftPosition::DEFAULT);
    intake.setRingSortColour(true);

    //Alliance
    intake.autonFunctions(12000);
    pros::delay(400);
    drivetrain.drive(32, 600, 12000, false);

    //Mogo 1
    drivetrain.turnToHeading(-90, 600, false, false);
    drivetrain.boomerang(Pose(60, 32, 90), 6000, 0.5, true, false, true);
    drivetrain.waitUntil(55);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Early ring
    drivetrain.turnToPoint(Point(60, 90), 600, false);
    drivetrain.boomerang(Pose(60, 90, 0), 6000, 0.5, false, false, false);
    drivetrain.stop(100);

    //Top ring
    drivetrain.turnToPoint(Point(120, 225), 600, false);
    drivetrain.boomerang(Pose(115, 225, 10), 6000, 0.5, false, false, true);
    drivetrain.waitUntil(80);
    lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Wall
    drivetrain.boomerang(Pose(100, 158.5, -135), 6000, 0.5, true, false, false);
    drivetrain.stop(100);
    intake.autonFunctions(-12000);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    drivetrain.turnToHeading(90, 600, false, false);
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(129, 158.5, 90), 6000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.drive(40, 800, 12000, true);
    pros::delay(400);
    lift.holdPosition(subsystems::LiftPosition::ALLIANCE);
    drivetrain.waitUntilEnd();
    intake.autonFunctions(12000);
    drivetrain.drive(-30, 600, 12000, false);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    
    //Strip
    drivetrain.turnToHeading(180, 600, false, false);
    drivetrain.boomerang(Pose(115, 50, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(115, 30, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(115, 10, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.drive(-30, 600, 12000, false);
    
    //Extra
    drivetrain.turnToHeading(75, 600, false, false);
    drivetrain.boomerang(Pose(125, 45, 75), 6000, 0.5, false, false, false);
    drivetrain.stop(100);
    
    //Corner
    drivetrain.turnToHeading(-20, 600, false, false);
    pros::delay(300);
    mogo.setState(false);
    drivetrain.drive(-30, 600, 12000, false);
    intake.autonFunctions(-12000);
    drivetrain.drive(20, 600, 12000, false);

    //2nd mogo
    lift.holdPosition(subsystems::LiftPosition::DEFAULT);
    drivetrain.turnToHeading(90, 600, false, false);
    drivetrain.boomerang(Pose(-70, 36, -90), 2000, 0.5, true, false, true);
    drivetrain.waitUntil(192);
    mogo.setState(true);
    drivetrain.waitUntilEnd();
    drivetrain.stop(300);

    //Early ring
    intake.autonFunctions(12000);
    drivetrain.turnToHeading(0, 600, false, false);
    drivetrain.boomerang(Pose(-73, 85, 0), 6000, 0.5, false, false, false);
    drivetrain.stop(100);

    //Top ring
    drivetrain.turnToHeading(-32, 600, false, false);
    drivetrain.boomerang(Pose(-129, 222, 0), 6000, 0.5, false, false, true);
    drivetrain.waitUntil(80);
    lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Wall
    drivetrain.boomerang(Pose(-118, 158, 135), 6000, 0.5, true, false, false);
    drivetrain.stop(100);
    intake.autonFunctions(-12000);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    drivetrain.turnToHeading(-90, 600, false, false);
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(-146, 158, -90), 6000, 0.2, false, false, false);
    drivetrain.stop(100);
    drivetrain.drive(40, 800, 12000, true);
    pros::delay(400);
    lift.holdPosition(subsystems::LiftPosition::ALLIANCE);
    drivetrain.waitUntilEnd();
    intake.autonFunctions(12000);
    drivetrain.drive(-30, 600, 12000, false);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    
    //Strip
    drivetrain.turnToHeading(180, 600, false, false);
    drivetrain.boomerang(Pose(-117, 50, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(-117, 30, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(-117, 10, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.drive(-30, 600, 12000, false);
    
    //Extra
    drivetrain.turnToHeading(-75, 600, false, false);
    drivetrain.boomerang(Pose(-137, 45, -75), 6000, 0.5, false, false, false);
    drivetrain.stop(100);
    pros::delay(350);
    
    //Corner
    drivetrain.turnToHeading(20, 600, false, false);
    pros::delay(300);
    mogo.setState(false);
    drivetrain.drive(-30, 600, 12000, false);
    intake.autonFunctions(-12000);
    drivetrain.drive(20, 600, 12000, false);
    drivetrain.stop(100);

    //Pickup extra ring
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(-62, 215, 45), 6000, 0.5, false, false, false);
    lift.holdPosition(subsystems::LiftPosition::DEFAULT);
    drivetrain.stop(100);

    //3rd mogo in corner
    drivetrain.boomerang(Pose(-35, 285, 0), 6000, 0.5, false, false, true);
    pros::delay(100);
    intake.autonFunctions(1000);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    drivetrain.turnToHeading(-90, 600, false, false);
    drivetrain.boomerang(Pose(-131, 317, -87), 8000, 0.5, false, false, false);
    drivetrain.stop(100);

    //4th mogo
    drivetrain.boomerang(Pose(-5, 275, 90), 3000, 0.5, true, false, false);
    mogo.setState(true);
    drivetrain.stop(100);

    //Extra ring
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(-100, 275, -90), 6000, 0.5, false, false, false);
    drivetrain.stop(100);

    //Middle ring
    drivetrain.turnToHeading(135, 600, false, false);
    drivetrain.stop(100);

    drivetrain.boomerang(Pose(-15, 177, 135), 6000, 0.5, false, false, true);
    drivetrain.waitUntil(40);
    intake.autonFunctions(0);
    drivetrain.waitUntil(80);
    intake.autonFunctions(12000);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Extra close ring
    drivetrain.turnToHeading(45, 600, false, false);
    drivetrain.boomerang(Pose(70, 235, 45), 6000, 0.5, false, false, false);
    drivetrain.stop(100);

    intake.endAutoTask();
    intake.setVoltage(12000);

    //mid corner ring
    drivetrain.boomerang(Pose(110, 282, 0), 3000, 0.5, false, false, false);
    intake.setVoltage(12000);

    drivetrain.stop(100);
    drivetrain.drive(-15, 500, 12000, false);

    //other corner ring
    drivetrain.boomerang(Pose(140, 290, 65), 3000, 0.5, false, false, false, 1300);
    drivetrain.stop(100);

    //Corner
    drivetrain.turnToHeading(-160, 1000, false, false);
    mogo.setState(false);
    drivetrain.drive(-40, 1000, 12000, false);
    intake.autonFunctions(-12000);


    //Ladder
    drivetrain.drive(30, 800, 12000, false);

    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    drivetrain.turnToHeading(45, 600, false, false);
    drivetrain.stop(100);

    intake.autonFunctions(0);

    drivetrain.boomerang(Pose(48, 205, -135), 1500, 0.5, true, false, false, 3000);
    drivetrain.drive(-20, 500, 6000, false);
    drivetrain.stop(100);
    drivetrain.drive(30, 500, 2000, false);
*/
}

void opcontrol() 
{
    intake.endAutoTask();
    intake.holdPosition(subsystems::DEFAULT);
    drivetrain.stopOdom();
    drivetrain.setBrakeMode(MOTOR_BRAKE_COAST);
    while(true)
    {
        //Controlling Drivetrain
        drivetrain.driverFunctions();
        //Controlling Intake/Lift
        intake.driverFunctions();
        //Controlling Mogo
        mogo.driverFunctions();
        //Controlling Doinkers
        doinkers.driverFunctions();
        //Controlling goal grabber
        goalGrabber.driverFunctions();
        
        pros::delay(10);
    }
}