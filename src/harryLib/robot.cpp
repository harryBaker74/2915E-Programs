#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/odom.hpp"
#include "../include/harryLibHeader/pid.hpp"
#include "../include/harryLibHeader/velocityController.hpp"
#include "../include/harryLibHeader/util.hpp"
#include "../include/harryLibHeader/boomerang.hpp"


//File for controlling all systems in the robot



namespace subsystems
{   

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Drivetrain Class
        
        //constructor
        drivetrain::drivetrain( int leftFrontMotorPort, int leftMidMotorPort, int leftBackMotorPort,
                                int rightFrontMotorPort, int rightMidMotorPort, int rightBackMotorPort, 
                                int trackingWheelPort, int inertialPort)

        :   leftFrontMotor(pros::Motor (leftFrontMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)), 
            leftMidMotor(pros::Motor (leftMidMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            leftBackMotor(pros::Motor (leftBackMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            rightFrontMotor(pros::Motor (rightFrontMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            rightMidMotor(pros::Motor (rightMidMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            rightBackMotor(pros::Motor (rightBackMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            trackingWheel(pros::Rotation(trackingWheelPort)),
            IMU(pros::IMU(inertialPort))
        {
            leftDriveMotors.append(leftMidMotor);
            leftDriveMotors.append(leftBackMotor);
            
            rightDriveMotors.append(rightMidMotor);
            rightDriveMotors.append(rightBackMotor);

        }

        /**
         * @brief Function to start running odomentery calculations. 
         * Does not need to be called as a task, it alrteady has lambda task in it.
         * 
         * @param startPose The pose the odometery calculations will start from;
         */
        void drivetrain::runOdom(Pose startPose)
        {   
            //Resetting position of all motor encoders and tracking wheels to zero
            leftDriveMotors.tare_position_all();
            rightDriveMotors.tare_position_all();
            trackingWheel.reset_position();

            //Setting pose to desired start pose
            this->pose = startPose;

            //making sure the task actually runs
            odomRunning = true;
            //Setting prev encoder values
            Odometery::prevEncoderValues =
                {
                    {
                        0, 0, 0
                    },
                    {
                        0, 0, 0
                    },
                    {
                        this->IMU.get_rotation() * M_PI / 180
                    }
                };

            pros::Task task{[=, this] {
                
                while(odomRunning)
                {
                //Setting previous pose to current pose
                this->prevPose = this->pose;
                
                //Updating current pose
                //Doing the Odometery Calculations and setting the class varibles for other functions to use
                //Passing in addresses to motor + rotation cause i couldn't figure out how to do it differently
                Odometery::OdometeryCalculations(&pose, &leftDriveMotors, &rightDriveMotors, &trackingWheel, &IMU);

                //Delay to give time for other tasks
                pros::delay(10);
                }

                //Stop Odom if its not supposed to be running
                pros::Task::current().remove();
            }};
        }

        /**
         * @brief Function to stop the odometery calculations.
         * If the Calculations already aren't running, nothing will happen.
         */
        void drivetrain::stopOdom()
        {
            odomRunning = false;
        }


        /**
         * @brief Function to set the voltage applied to all drive motors
         * 
         * @param left Voltage to left side of the drive(-12000 - 12000)
         * @param right Voltage to right side of the drive(-12000 - 12000)
         */
        void drivetrain::setVoltage(double left, double right)
        {
            //Setting Left Motors
            leftFrontMotor.move_voltage(floor(left));
            leftMidMotor.move_voltage(floor(left));
            leftBackMotor.move_voltage(floor(left));

            //Setting Right Motors
            rightFrontMotor.move_voltage(floor(right));
            rightMidMotor.move_voltage(floor(right));
            rightBackMotor.move_voltage(floor(right));
        }

        /**
         * @brief Function that handles controller inputs for drivetrain. Intended to be used during Op Control
         * 
         */
        void drivetrain::driverFunctions()
        {
            int leftJoystick = Controller.get_analog(ANALOG_LEFT_Y);
            int rightJoystick = Controller.get_analog(ANALOG_RIGHT_Y);

            int leftOutput = linearToCubed(leftJoystick, 127, 1);
            int rightOutput = linearToCubed(rightJoystick, 127, 1);

            this->setVoltage(leftOutput * 12000/127, rightOutput * 12000/127);
        }

        //////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////
        //AUTON FUNCTIONS

        void drivetrain::turnToHeading(double heading, bool radians, bool async)
        {   
            //Prevent this motion from starting if the robot is already in a motion
            while(inMotion)
                pros::delay(10);
            
            //Run function in a task if it is supposed to be async
            if (async)
            {
                pros::Task task {[=, this] {
                    turnToHeading(heading, radians, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {
                this->distanceTraveled = 0;
                this->inMotion = true;

                //TUNING
                    
                    //Guide to tuning
                    //Setup
                    //Set all pid variables to zero, and comment out the break in the exit condition if statement
                    //Run this function whenever you change the constants and take the error value to graph(print it or something)
                    //Give the function something like 90 degrees everytime you run so it actually has time to work
                    //Alrtenatively, give it random targets each time so your constants work better over multiple different target ranges
                    //
                    //Pid Tuning
                    //Start off by increasing Kp until you get 2-4 oscillations
                    //Then increase kd until the oscilattions stop, the graph should look like one sloped line curving into a straight horizontal line
                    //Set windup range to a bit above the current steady state error
                    //Set max integral to whatever you want honestly you prob dont need it
                    //Start increasing Ki very slowly until steady state error dissapears quickly.
                    //
                    //At this point un-comment the break in the exit condition if statement
                    //
                    //Exit Conditions Tuning
                    //(Note these values are in rad, rad/s, and rad/(s*s), so convert from deg if needed)
                    //Set the error exit to something like 0.1 degrees if your confident, or 0.25 if your not. Maybe go even lower if its really good
                    //Change around vel exit and accel exit to see what works best.(Aut Student Jordan D'Souza)
                    //In theory, vel exit should be about 1/100 error exit, and accel exit should be 1/10000 of error exit. but this might be wrong idk

                    //Pid for turning 
                    PID::PID pid(
                        650.0,    //Kp
                        120.0,    //Ki
                        6500.0,    //Kd
                        0.06,    //Windup Range
                        0.0     //Max Intergal
                    );

                    //Exit conditions
                    double errorExit = 0.015;
                    double velExit = 0.01;


                //Everything else

                //Velocity controller instance
                vController::vController vCon(false);

                double angle = heading;
                double prevVel = 0;
                int counter = 0;
                
                if(!radians) angle *= M_PI / 180;  //Converting to radians if needed

                //Finding the target rotation
                //Gets the difference between the desired bounded heading and the current bounded heading, then bounds this angle, then gets the current rotation added to it
                //In a roundabout way because we dont want to do turns bigger than 180 deg/1 pi, and its easier to track because the values cant overflow due to using rotation
                double targetRotation = pose.rotation + (boundAngle(angle - pose.heading, true));

                while(true)
                {
                    //Pid Velocity Calculations
                    double velocity = pid.getPid(pose.rotation, targetRotation);

                    //Getting current motor velocity
                    std::vector <double> leftVels = leftDriveMotors.get_actual_velocity_all();
                    double averageVel = (leftVels.at(0) + leftVels.at(1) + leftVels.at(2)) / 3;
                    
                    //Velocity controller converting desired motor velocity into voltage
                    double voltage = vCon.rpmVelToVoltage(averageVel, prevVel, velocity);  
                    
                    //Setting drivetrain voltage based on Pid Output, Slew, and Velocity controller output
                    this->setVoltage(voltage, -voltage);

                    //Exit Conditions
                    //Error, and Velocity based(only exit when both are low) so we dont need to worry about waiting with low error for a certain amount of time
                    //Because error is in rad: vel is rad/s
                    double error = pid.getError();
                    double errorVel = pid.getDervative() / 0.01; //Derivative is already just the rate of change

                    //Checking if all these values are below a certain threshold
                    if (fabs(error) < errorExit)
                    {   
                        //In 2 if statements so that it will only need to do a simple comparison if the error is too big,
                        //rather than 2 comparisons every loop.
                        if(fabs(errorVel) < velExit)
                            break; //Comment this break out if tuning
                    }

                    //Debug Printing
                    /*
                    printf("(%d, %f)\n", counter, error);
                    counter++;
                    */

                    //Updating distance traveled for async functions
                    this->distanceTraveled += this->pose.rotation - this->prevPose.rotation;
                    
                    //Delay for scheduling
                    pros::delay(10);
                }
                this->inMotion = false;
                this->setVoltage(0, 0);
            };
        }

        void drivetrain::moveToPoint(Point point, bool backwards, bool async)
        {
            //Prevent this motion from starting if the robot is already in a motion
            while(inMotion)
                pros::delay(10);


            if(async)
            {
                pros::Task task {[=, this] {
                    moveToPoint(point, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {
                this->distanceTraveled = 0;
                this->inMotion = true;

                PID::PID angPid = PID::PID(
                    2000.0,    //Kp
                    0.0,    //Ki
                    6000.0,    //Kd
                    0.0,    //Windup Range
                    0.0     //Max Intergal
                );
                PID::PID linPid = PID::PID(
                    10.0,
                    0.0,
                    125.0,
                    0.0,
                    0.0
                );

                //Exit conditions
                double errorExit = 1.75;
                double velExit = 10;

                //Angular Falloff Parameter
                //Smaller number means more abrupt falloff, with 0 being no fall off
                //Angular speed will be half when distance is at angK.
                //Eg:   angK = 1, angVel *= 0.5 for hypot = 1;
                //      angK = 2, angVel *= 0.5 for hypot = 2;
                //      angK = 3: angVel *= 0.5 for hypot = 3;
                //      etc.
                double angK = 10;

                vController::vController leftVCon(true);
                vController::vController notLeftVCon(true);


                //Prev velocities for velocity controllers
                double prevLeftVel = 0;
                double prevRightVel = 0;

                int linMultiplier = backwards ? -1 : 1;

                int counter = 0;

                while(true)
                {

                //Calculating delta cartesian offesets
                double deltaX = point.x - pose.x;
                double deltaY = point.y - pose.y;

                //Converts delta cartesian coordinates to polar coordinates, than takes theta and adds pi/2 to it to convert it to +y = 0, then bounds the angle;
                double targetHeading = atan3(deltaY, deltaX);

                targetHeading = backwards ? boundAngle(targetHeading + M_PI, true) : targetHeading;

                double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);

                //Calculating distance to point
                double hypot = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

                //Calculating angular velocity multiplier
                //Makes angular velocity weaker the closer you are to the point, eventually becoming zero when on the point
                //Stops spinning in circles, idea from Genesis/Daniel
                double angMultiplier = fabs(hypot) / (fabs(hypot) + angK);

                //Calculating velocities
                double angVel = angPid.getPid(pose.rotation, targetRotation) * angMultiplier;
                double linVel = linPid.getPid(hypot) *  linMultiplier;
                double leftVel = linVel + angVel;
                double rightVel = linVel - angVel;

                //Debug prints
                printf("(%d, %f)\n", counter, linPid.getError());
                counter++;

                double currentLeftVel = leftFrontMotor.get_actual_velocity();
                double currentRightVel = rightFrontMotor.get_actual_velocity();

                //Converting Velocities to voltage
                double leftVoltage = leftVCon.rpmVelToVoltage(currentLeftVel, prevLeftVel, leftVel);
                double rightVoltage = notLeftVCon.rpmVelToVoltage(currentRightVel, prevRightVel, rightVel);

                prevLeftVel = leftVel;
                prevRightVel = rightVel;

                this->setVoltage(leftVoltage, rightVoltage);

                //Exit Conditions, Semi circle exit
                //Should add velocity exit here in the future
                if(boomerang::semiCircleCheck(this->pose, point, this->pose.heading , errorExit))
                    break;

                //Updating distance traveled for async functions
                this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));

                //Delay for other tasks
                pros::delay(10);
                }
                this->inMotion = false;
                this->setVoltage(0, 0);
            }
        }

        void drivetrain::moveToPose(Pose targetPose, double dLead, double gLead, bool backwards, bool radians, bool async)
        {
            //Prevent this motion from starting if the robot is already in a motion
            while(inMotion)
                pros::delay(10);


            if (async)
            {
                pros::Task task {[=, this] {
                    moveToPose(targetPose, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {   
                this->distanceTraveled = 0;
                this->inMotion = true;

                //PID's
                PID::PID angPid = PID::PID(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                );

                PID::PID linPid = PID::PID(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                );

                //Switch conditions
                //The distance at which the robot needs to be from the current target point, in order to move onto the next target point
                double switchDistance = 5;
                //Radius of exit semicircle, prob should be automatic
                double exitDistance = 10;
                double velExit = 0;

                //Angular Falloff Parameter
                double angK = 1;

                //Angular switch parameter
                //This parameter decides when to switch from facing the carrot point to facing the heading
                //Lower values means that the switch will happen quicker, with 0 being no switch
                //Eg:   angSwitch = 1, 50, 50 weighted for hypot = 1;
                //      angSwitch = 2, 50, 50 weighted for hypot = 2;
                double angSwitch = 1;

                //Converting to radians if needed
                if(!radians) targetPose.heading *= M_PI / 180;  //Converting to radians if needed

                //Caluclating initial carrot point
                Point initialCarrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                //Initialixing carrot point
                Point carrot = initialCarrot;

                //Switch boolean
                bool followGhost = true;

                //Initializing Velocity Controllers
                vController::vController leftVCon(true);
                vController::vController notLeftVCon(true);

                //Prev velocities for velocity controllers
                double prevLeftVel = 0;
                double prevRightVel = 0;

                //Updating varibales for bakwards movement
                int linMultiplier = backwards ? -1 : 1;
                targetPose.heading = backwards ? boundAngle(targetPose.heading + M_PI, true) : targetPose.heading;
                
                while(true)
                {  
                    //Deciding what carrot point to use
                    //Inspired by https://github.com/Pixel-Lib/Pixel/blob/main/src/pxl/movements/boomerang.cpp
                    if(followGhost)
                    {
                        carrot = boomerang::getGhost(initialCarrot, carrot, gLead);
                    }
                    else
                    {
                        carrot = boomerang::getCarrot(this->pose, targetPose, dLead);
                    }

                    //Calculating offset from carrot point
                    Point deltaPos(carrot.x - this->pose.x, carrot.y - this->pose.y);

                    //Converts delta cartesian coordinates to polar coordinates, than takes theta and adds pi/2 to it to convert it to +y = 0, then bounds the angle to -pi/pi;
                    double targetHeading = atan3(deltaPos.y, deltaPos.x);
                    //Adding 180 deg to target heading if we're driving backwards
                    targetHeading = backwards ? boundAngle(targetHeading + M_PI, true) : targetHeading;
                    
                    //Checking if the robot has reached the ghost point, indicating it to switch target to the end point
                    if(boomerang::semiCircleCheck(this->pose, initialCarrot, targetHeading, switchDistance))
                    {
                        followGhost = false;
                    }

                    //Caluclating distance to carrot
                    double distance = sqrt(pow(deltaPos.x, 2) + pow(deltaPos.y, 2));

                    //Figures out the nearest multiple of the difference between the target heading and current heading, to the current rotation
                    double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);
                    //Setting the target pose' rotation the same way
                    targetPose.rotation = pose.rotation + boundAngle(targetPose.heading - pose.heading, true);
                    
                    //Setting the target rotation to a weighted average between the rotation to the carrot, and the end target rotation
                    //Based off of angSwitch;
                    targetRotation = getWeightedAverage(targetRotation, targetPose.rotation, fabs(distance) / (fabs(distance) + angSwitch));

                    //Calculating angMultiplier
                    //Lowers the angular velocity the closer you get to the end point, preventing spinning in circles
                    double angMultiplier = fabs(distance) / (fabs(distance) + angK);

                    //Calculate velocities
                    double angVel = angPid.getPid(this->pose.rotation, targetRotation) * angMultiplier;
                    double linVel = linPid.getPid(distance) * linMultiplier;
                    double leftVel = linVel + angVel;
                    double rightVel = linVel - angVel;

                    //Getting current velocities
                    double currentLeftVel = leftFrontMotor.get_actual_velocity();
                    double currentRightVel = rightFrontMotor.get_actual_velocity();

                    //Converting Velocities to voltage
                    double leftVoltage = leftVCon.rpmVelToVoltage(currentLeftVel, prevLeftVel, leftVel);
                    double rightVoltage = notLeftVCon.rpmVelToVoltage(currentRightVel, prevRightVel, rightVel);
                    //Updating prev Velocities
                    prevLeftVel = leftVel;
                    prevRightVel = rightVel;

                    //Setting the motors voltage
                    this->setVoltage(leftVoltage, rightVoltage);

                    //Exit Conditions, Semi circle exit
                    //Should add velocity exit here in the future
                    if(boomerang::semiCircleCheck(this->pose, Point(targetPose.x, targetPose.y), targetPose.heading, exitDistance))
                        break; //Comment out for tuning maybe


                    //Updating distance traveled for async functions
                    this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));
                    
                    //Delay for other tasks
                    pros::delay(10);
                }
                this->inMotion = false;
                this->setVoltage(0, 0);
            }
        }












/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int bottomIntakeMotorPort, int topIntakeMotorPort)
        :   bottomIntakeMotor(pros::Motor (bottomIntakeMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            topIntakeMotor(pros::Motor (topIntakeMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees))
        {}

        //Function to set intake voltage
        void intake::setVoltage(double voltage)
        {
            bottomIntakeMotor.move_voltage(floor(voltage));
            topIntakeMotor.move_voltage(floor(voltage));
        }

        //Function to run intake during driver control
        void intake::driverFunctions()
        {
            setVoltage((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2)) * 12000);
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Basket Class
        //Constructor
        basket::basket(int basketLeftMotorPort, int basketRightMotorPort)
        :   basketLeftMotor(pros::Motor (basketLeftMotorPort, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees)),
            basketRightMotor(pros::Motor (basketRightMotorPort, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees))
        {

            basketLeftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            basketRightMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
        }

        //Function to set basket voltage
        void basket::setVoltage(double voltage)
        {
            basketLeftMotor.move_voltage(floor(voltage));
            basketRightMotor.move_voltage(floor(voltage));
        }

        //Function to run basket during driver control
        void basket::driverFunctions()
        {
            setVoltage((Controller.get_digital(DIGITAL_A) - Controller.get_digital(DIGITAL_B)) * 12000);
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Mogo class
        //Constructor
        mogo::mogo(char mogoSolanoidPort)
        :   mogoSolanoid(pros::adi::Pneumatics (mogoSolanoidPort, false, false))
        {}

        //Function to set mogo output
        void mogo::setState(bool state)
        {
            mogoSolanoid.set_value(state);
        }

        //Function to run mogo during driver control
        void mogo::driverFunctions()
        {
            mogoPressCount += Controller.get_digital_new_press(DIGITAL_L1);
            mogoPressCount % 2 == 0 ? setState(false) : setState(true);
        }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}