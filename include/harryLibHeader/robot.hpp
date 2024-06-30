#pragma once
#include "../main.h"
#include "util.hpp"



//File for controlling all systems in the robot
    

namespace subsystems
{
    class drivetrain
    {   

        pros::Motor leftFrontMotor;
        pros::Motor leftMidMotor;
        pros::Motor leftBackMotor;
        
        pros::Motor rightFrontMotor;
        pros::Motor rightMidMotor;
        pros::Motor rightBackMotor;

        pros::MotorGroup leftDriveMotors = pros::MotorGroup(leftFrontMotor);
        pros::MotorGroup rightDriveMotors = pros::MotorGroup(rightFrontMotor);

        pros::Rotation trackingWheel;

        pros::IMU IMU;

        Pose pose = Pose(0, 0, 0);

        bool odomRunning = false;


        public:
        //constructor
        drivetrain(int leftFrontMotorPort, int leftMidMotorPort, int leftBackMotorPort,
                   int rightFrontMotorPort,int rightMidMotorPort,int rightBackMotorPort,
                   int trackingWheelPort, int inertialPort);

        //Driver
        /**
         * @brief Function that handles controller inputs for drivetrain. Intended to be used during Op Control
         * 
         */
        void driverFunctions();

        //Helper Functions
        /**
         * @brief Function to set the voltage applied to all drive motors
         * 
         * @param left Voltage to left side of the drive(-12000 to 12000)
         * @param right Voltage to right side of the drive(-12000 to 12000)
         */
        void setVoltage(double left, double right);

        void runOdom(Pose startPose);

        void stopOdom();

        //////////////////////////////////////////////////////////////////////////////////////////
        //AUTON FUNCTIONS
        /**
        * @brief Turns the robot on a point to face a direction
        */
        void turnToHeading(double heading, bool radians);
        /**
         * @brief Function to move the robot from its current pose to a point. First turns, then drives
         */
        void moveToPoint(Point point);
        /**
         * @brief Function to move the robot from its current pose to the desired pose
         * Uses a Boomerang controller
         */
        void moveToPose();

        /**
         * @brief Function to make the robot follow a desired path from the path generator or path scheduler
         */
        void followPath();

    };

    class intake
    {
        pros::Motor intakeMotor;

        public:
        //Constructor
        intake(int intakeMotorPort);

        //Function to set intake voltage
        void setVoltage(double voltage);

        //Function to run intake during driver control
        void driverFunctions();
    };

    class plunger
    {
        pros::Motor plungerMotor;
        pros::adi::Pneumatics armPiston;
        pros::adi::Pneumatics clampPiston;

        int armPressCount = 0;
        int clampPressCount = 0;

        public:
        //Constructor
        plunger(int plungerMotorPort, char armPistonPort, char clampPistonPort);

        //Function to set plunger voltage
        void setVoltage(double voltage);

        void setArmState(bool state);
        void setClampState(bool state);

        //Function to run plunger during driver control
        void driverFunctions();
    };

    class mogo
    {
        pros::adi::Pneumatics mogoSolanoid;

        int mogoPressCount = 0;

        public:
        //Constructor
        mogo(char mogoSolanoidPort);

        //Function to set mogo output
        void setState(bool state);

        //Function to run mogo during driver control
        
        void driverFunctions();
    };
}