#pragma once
#include "../main.h"
#include "util.hpp"
#include "pathGen.hpp"


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

        //Max voltage rate of change per ms, used for a few algorithms
        double voltageSlew = 40;

        double prevLeftVoltage = 0;
        double prevRightVoltage = 0;


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
         * @param slew Whether or not to limit the rate of change of the voltage, aimed at preventing wheel slippage
         * @param timestep The timestep between this call, and the previous call. Only matters if using slew
         */
        void setVoltage(double left, double right, bool doSlew = false, double timestep = 10);

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

        void stop(int timeMs = 0);

        /**
        * @brief Turns the robot on a point to face a direction
        */
        void turnToHeading(double heading, int timeout_ms, bool radians = false, bool async = true);

        /**
         * @brief Function to drive the robot 1 dimensionally, a certain distance
         */
        void drive(double distance, double maxVoltage = 12000, bool async = true);
        /**
         * @brief Function to move the robot from its current pose to a point. First turns, then drives
         */
        void moveToPoint(Point point, bool backwards = false, bool async = true);
        /**
         * @brief Function to move the robot from its current pose to the desired pose
         * Uses a Boomerang controller
         */
        void boomerang(Pose pose, double minSpeed, double dLead, bool backwards, bool radians = false, bool async = true);

        /**
         * @brief Function that makes the robot follow an inputted cubic Bezier curve and linear motion profile
         */
        void tangentIntersection(cubicBezier curve, std::vector<std::vector<double>> profile, bool backwards = false, bool async = true);

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

    enum LiftPosition
    {
        DEFAULT = 35,
        WALLSTAKESCORE = 340,
        WALLSTAKEDROP = 200,
        ALLIANCESTAKESCORE = 240,
        ALLIANCESTAKEDROP = 0
    };

    class basket
    {

        enum LiftPosition targetPos = DEFAULT;
        bool hold = false;

        pros::Motor basketMotor;
        pros::adi::Pneumatics basketPistons;

        int basketPressCount = 0;

        public:
        //Constructor
        basket(int basketMotorPort, char basketPistonsPort);

        //Function to set basket voltage
        void setVoltage(double voltage);
        //Function to set piston state
        void setState(bool state);


        //Functions to move lift to certain positions
        void setPosition(enum LiftPosition position);
        void holdPosition(enum LiftPosition position);
        void endHold();
        //Function to convert current encoder position into an amount of radians for the lift in real life. Used for vel controller
        double encoderToRad(double encoder);

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