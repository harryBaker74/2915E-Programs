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
    subsystems::lift lift = subsystems::lift(LIFT_1, LIFT_2, ROTATION);
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
//Red side goal rush
/*
    drivetrain.runOdom(Pose(0, 0, 0));
    intake.setRingSortColour(true);
    lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

    drivetrain.boomerang(Pose(-5, 85, -10), 8000, 0.5, false, false, true);
    drivetrain.waitUntil(20);
    intake.autonFunctions(12000);
    drivetrain.waitUntil(80);
    rushMech.setState(true);
    drivetrain.waitUntilEnd();
    
    drivetrain.stop(100);

    drivetrain.boomerang(Pose(5, 85, 170), 8000, 0.5, true, false, false);
    drivetrain.stop(100);
*/


//blue side alliance + 4 + ladder
    /*
        drivetrain.runOdom(Pose(0, 0, -90));
        intake.setRingSortColour(false);
        lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);

        //Alliance stake
        drivetrain.turnToHeading(-122, 700, false, false);
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
        drivetrain.boomerang(Pose(113, 2, 135), 6000, 0.5, false, false, false);
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
        drivetrain.turnToHeading(-122, 700, false, false);
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
        drivetrain.turnToHeading(122, 500, false, false);
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

        //Getting 2 mid rings
        drivetrain.turnToHeading(0, 1000, false, false);
        drivetrain.boomerang(Pose(-85, 126, -92), 3000, 0.6, false, false, false);
        drivetrain.drive(20, 500, 8000, false);

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
    drivetrain.boomerang(Pose(100, 162, -135), 6000, 0.5, true, false, false);
    drivetrain.stop(100);
    intake.autonFunctions(-12000);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    drivetrain.turnToHeading(90, 600, false, false);
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(129, 162, 90), 6000, 0.5, false, false, false);
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
    drivetrain.boomerang(Pose(105, 50, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(105, 30, -180), 3000, 0.5, false, false, false);
    drivetrain.stop(100);
    drivetrain.boomerang(Pose(105, 10, -180), 3000, 0.5, false, false, false);
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
    drivetrain.boomerang(Pose(-132, 222, 0), 6000, 0.5, false, false, true);
    drivetrain.waitUntil(80);
    lift.holdPosition(subsystems::LiftPosition::AUTOLOAD);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Wall
    drivetrain.boomerang(Pose(-119, 162, 135), 6000, 0.5, true, false, false);
    drivetrain.stop(100);
    intake.autonFunctions(-12000);
    lift.holdPosition(subsystems::LiftPosition::DOUBLERING);
    drivetrain.turnToHeading(-90, 600, false, false);
    intake.autonFunctions(12000);
    drivetrain.boomerang(Pose(-146, 162, -90), 6000, 0.5, false, false, false);
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
    intake.autonFunctions(1000);
    drivetrain.stop(100);

    //3rd mogo in corner
    drivetrain.boomerang(Pose(-34, 282, 0), 6000, 0.5, false, false, false);
    drivetrain.stop(100);

    drivetrain.turnToHeading(-90, 600, false, false);
    drivetrain.boomerang(Pose(-131, 312, -87), 8000, 0.5, false, false, false);
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

    drivetrain.boomerang(Pose(-10, 172, 135), 6000, 0.5, false, false, true);
    drivetrain.waitUntil(40);
    intake.autonFunctions(0);
    drivetrain.waitUntil(80);
    intake.autonFunctions(12000);
    drivetrain.waitUntilEnd();
    drivetrain.stop(100);

    //Extra close ring
    drivetrain.turnToHeading(45, 600, false, false);
    drivetrain.boomerang(Pose(75, 220, 45), 6000, 0.5, false, false, false);
    drivetrain.stop(100);

    intake.endAutoTask();
    intake.setVoltage(12000);

    //mid corner ring
    drivetrain.boomerang(Pose(110, 280, 45), 3000, 0.5, false, false, false);
    intake.setVoltage(12000);

    drivetrain.stop(100);
    drivetrain.drive(-10, 300, 12000, false);

    //other corner ring
    drivetrain.turnToHeading(90, 600, false, false);
    drivetrain.boomerang(Pose(120, 280, 90), 3000, 0.5, false, false, false);
    drivetrain.stop(100);





}

void opcontrol() 
{
    intake.endAutoTask();
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