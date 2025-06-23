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

        

        bool odomRunning = false;

        bool inMotion = false;
        double distanceTraveled = 0;

        //Max voltage rate of change per ms, used for a few algorithms
        double voltageSlew = 40;

        double prevLeftVoltage = 0;
        double prevRightVoltage = 0;


        public:

        Pose pose = Pose(0, 0, 0);
        Pose prevPose = pose;

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

        /**
         * @brief Sets the pose of the robot
         */
        void setPose(Pose pose);

        /**
         * @brief Set the x and y position of the robot, not affecting heading or rotation
         */
        void setPose(Point point);

        void setBrakeMode(enum pros::motor_brake_mode_e brakeMode);

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
         * @brief Turns the robot to a point
         */
        void turnToPoint(Point point, int timeout_ms, bool async = true);

        /**
         * @brief Function to drive the robot 1 dimensionally, a certain distance
         */
        void drive(double distance, double timeout_ms, double maxVoltage = 12000, bool async = true);
        
        /**
         * @breif drives the distance to the point from current position
         */
        void driveToPoint(Point point, double timeout_ms, double maxVoltage = 12000, bool async = true);
        /**
         * @brief Function to move the robot from its current pose to a point. First turns, then drives
         */
        void moveToPoint(Point point, bool backwards = false, bool async = true);
        /**
         * @brief Function to move the robot from its current pose to the desired pose
         * Uses a Boomerang controller
         */
        void boomerang(Pose pose, double maxSpeed, double dLead, bool backwards, bool radians = false, bool async = true, int timeout_ms = 10000);

        /**
         * @brief Function that makes the robot follow an inputted cubic Bezier curve and linear motion profile
         */
        void tangentIntersection(cubicBezier curve, std::vector<std::vector<double>> profile, bool backwards = false, bool async = true);

        /**
         * @brief Function to make the robot swing(only move one side of the drivetrain) to a desired heading
         * 
         * @param side Side of drivetrain to move, true for left
         * @param backwards Direction to move drivetrain side to reach heading
         */
        void swingToHeading(double heading, bool side, bool backwards = false, bool radians = false, bool async = true, int timeout_ms = 100000);

        /**
         * @brief Function to make the robot follow a desired trajectory from the path generator
         * Uses a ramsete controller
         * 
         * @param path Trajectory to follow
         * @param lookaheadDist The distance to lookahead in the path when trying to find the closest point.
         * Small lookahead might act strangely and slow down path following.
         * Large lookahead might cause robot to skip when path crosses over itself, and will take more resources
         */
        void followPath(trajectory path, double lookaheadDist);

    };

    enum LiftPosition
    {
        DEFAULT = -15,
        AUTOLOAD = 40,
        LOAD = 40,
        DOUBLERING = 125,
        WALL = 237,
        ALLIANCE = 249,
        AUTOALLIANCESLAM =  305,
        AUTOALLIANCE = 275,
        GOALRUSH = 265,
        TIP = 365,
        GOALLEAVE = 205,
        ZERO = 110
    };

    class intake
    {
        pros::Motor intakeMotor;
        pros::adi::Pneumatics intakeLift;
        pros::Optical intakeOptical;

        pros::Motor liftMotor;
        pros::Rotation rotationSensor;
        pros::Optical liftOptical;


        int intakeLiftPressCount = 0;
        bool sortColour = false;
        bool sorting = false;
        bool allianceRing = false;
        double sortStartTime = 0;
        double sortStartTimeOffset = 25;
        double sortTime = 100;
        double sortEndTime = 0;
        int redMin = 0;
        int redMax = 20;
        int blueMin = 200;
        int blueMax = 250;

        int antiJamEndTime = 0;

        bool colourSortingOn = true;
        int colourSortingPressCount = 0;

        bool holding = false;
        LiftPosition targetPos = DEFAULT;
        int liftPressCount = 0;
        bool wall = false;
        bool alliance = false;
        bool tip = false;
        double startingPosition = 0;



        double autonVoltage = 0;
        bool auton = false;
        
        public:
        //Constructor
        intake(int intakeMotorPort, int intakeOpticalPort, char intakeLiftSolanoidPort, int liftMotorPort, int rotationSensorPort, int liftOpticalPort);

        //Function to set intake voltage
        void setIntakeVoltage(double voltage);

        //Function to set intake lift piston state
        void setIntakeLiftState(bool state);

        //Function to set ring colour being sorted out
        /**
         * @param colour True for sorting out blue, false for sorting out red
         */
        void setRingSortColour(bool colour);

        /**
         * @brief Sets the colour sorting to be on or off
         */
        void setColourSortState(bool state);

        /**
         * @brief Waits for the ring thats not being sorted to be detected
         */
        void waitForRing(int timeout_ms = 10000);

        //Function to run intake during driver control
        void driverFunctions();

        //Function to run intake during auton
        void autonFunctions(double voltage);

        void endAutoTask();

        //Function to set lift voltage
        void setLiftVoltage(double voltage);

        //Sets the sarting position for the lift
        void setLoadStartingPosition();

        //Holds the lift at a certain angle
        void holdPosition(LiftPosition pos);

    };

    class lift
    {
        pros::Motor liftMotor;

        pros::Rotation rotationSensor;

        bool holding = false;
        LiftPosition targetPos = DEFAULT;
        int pressCount = 0;
        bool wall = false;
        bool alliance = false;
        bool tip = false;

        public:
        //Constructor
        lift(int liftMotorPort, int rotationSensorPort);

        

        //Function for driver control
        void driverFunctions();
    };

    class mogo
    {
        pros::adi::Pneumatics mogoSolanoid;
        pros::Optical opticalSensor;

        int hueMin = 58;
        int hueMax = 67;

        bool holding = false;
        bool primed = true;
        int startCheckTime = 0;

        public:
        //Constructor
        mogo(char mogoSolanoidPort, int mogoOpticalPort);

        //Function to set mogo output
        void setState(bool state);

        //Function to run mogo during driver control
        void driverFunctions();
    };

    class doinkers
    {
        pros::adi::Pneumatics leftDoinker;
        pros::adi::Pneumatics rightDoinker;

        int leftPressCount = 0;
        int rightPressCount = 0;

        public:
        //Constructor
        doinkers(char leftDoinkerSolanoidPort, char rightDoinkerSolanoidPort);

        //Function to set states
        void setStates(bool left, bool right);

        //Function for driver control
        void driverFunctions();
    };

    class goalGrabber
    {
        pros::adi::Pneumatics goalGrabberSolanoid;

        int pressCount = 1;

        public:
        //Constructor
        goalGrabber(char goalGrabberSolanoidPort);

        //Setting state
        void setState(bool state);

        //Driver control funtions
        void driverFunctions();
    };
}