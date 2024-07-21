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
        Pose prevPose = pose;

        bool odomRunning = false;

        bool inMotion = false;
        double distanceTraveled = 0;


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
         * @brief Function that returns after a certain distance has been covered, or once a motion is done
         * @param distance The distance to return at, in Centimeters from target point for 2d movements, and radians for 1d turning
        */
        void waitUntil(double distance);
        /**
         * @brief Function that returns once a motion is done
        */
        void waitUntilEnd();
        /**
        * @brief Turns the robot on a point to face a direction
        */
        void turnToHeading(double heading, bool radians = false, bool async = true);
        /**
         * @brief Function to move the robot from its current pose to a point. First turns, then drives
         */
        void moveToPoint(Point point, bool backwards = false, bool async = true);
        /**
         * @brief Function to move the robot from its current pose to the desired pose
         * Uses a Boomerang controller
         */
        void moveToPose(Pose pose, double dLead = 0.5, double gLead = 0.5, bool backwards, bool radians = false, bool async = true);

        /**
         * @brief Function to make the robot follow a desired path from the path generator or path scheduler
         */
        void followPath();

    };

    class intake
    {
        pros::Motor topIntakeMotor;
        pros::Motor bottomIntakeMotor;

        public:
        //Constructor
        intake(int bottomIntakeMotorPort, int topIntakeMotorPort);

        //Function to set intake voltage
        void setVoltage(double voltage);

        //Function to run intake during driver control
        void driverFunctions();
    };

    class basket
    {
        pros::Motor basketLeftMotor;
        pros::Motor basketRightMotor;

        public:
        //Constructor
        basket(int basketLeftMotorPort, int basketRightMotorPort);

        //Function to set basket voltage
        void setVoltage(double voltage);

        //Function to run basket during driver control
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