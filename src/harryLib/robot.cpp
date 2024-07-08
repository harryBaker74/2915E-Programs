#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/odom.hpp"
#include "../include/harryLibHeader/pid.hpp"
#include"../include/harryLibHeader/velocityController.hpp"
#include "../include/harryLibHeader/util.hpp"

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

        :   leftFrontMotor(pros::Motor (leftBackMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)), 
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

            pros::Task task{[=, this] {
                
                while(odomRunning)
                {
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

            setVoltage(leftOutput, rightOutput);
        }

        //////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////
        //AUTON FUNCTIONS

        void drivetrain::turnToHeading(double heading, bool radians, bool async)
        {
            if (async)
            {
                pros::Task task {[=, this] {
                    turnToHeading(heading, radians, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {      
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
                        0.0,    //Kp
                        0.0,    //Ki
                        0.0,    //Kd
                        0.0,    //Windup Range
                        0.0     //Max Intergal
                    );

                    //Exit conditions
                    double errorExit = 0;
                    double velExit = 0;


                //Everything else

                //Velocity controller instance
                vController::vController vCon;

                double angle = heading;
                double prevOutput; //For slew
                
                if(!radians) angle *= M_PI / 180;  //Converting to radians if needed

                //Finding the target rotation
                //Gets the difference between the desired bounded heading and the current bounded heading, then bounds this angle, then gets the current rotation added to it
                //In a roundabout way because we dont want to do turns bigger than 180 deg/1 pi, and its easier to track because the values cant overflow due to using rotation
                double targetRotation = pose.rotation + (boundAngle(angle - pose.heading, true));

                //Static Variables for Exit conditions

                while(true)
                {
                    //Pid Velocity Calculations
                    double output = pid.getPid(pose.rotation, targetRotation);
                    //Giving a slew(rate limiter) to the output, so we dont accel to fast
                    double velocity = slew(output, prevOutput, MAX_RPMPS / 1000, 10);
                    prevOutput = output;


                    //Getting current motor velocity
                    std::vector <double> leftVels = leftDriveMotors.get_actual_velocity_all();
                    double averageVel = (leftVels.at(0) + leftVels.at(1) + leftVels.at(2)) / 3; //Not divided by 2 becasue the slew is standard
                    //Velocity controller converting desired motor velocity into voltage
                    double voltage = vCon.rpmVelToVoltage(averageVel, velocity);  
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
                        //rather than 2 comparisons every loop. Its called weight saving
                        if(fabs(errorVel) < velExit)
                            break; //Comment this break out if tuning
                    }
                    
                    //Delay for scheduling
                    pros::delay(10);
                }
                this->setVoltage(0, 0);
                
            };
        }

        void drivetrain::moveToPoint(Point point, bool async)
        {
            if(async)
            {
                pros::Task task {[=, this] {
                    moveToPoint(point, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {
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

                    //Exit conditions
                    double errorExit = 0;
                    double velExit = 0;

                vController::vController leftVCon;
                vController::vController notLeftVCon;

                double prevLeftVel;
                double prevRightVel;

                while(true)
                {

                //Calculating delta cartesian offesets
                double deltaX = point.x - pose.x;
                double deltaY = point.y - pose.y;

                //Converts delta cartesian coordinates to polar coordinates, than takes theta and adds pi/2 to it to convert it to +y = 0, then bounds the angle;
                double targetHeading = boundAngle(M_PI_2 + atan2(deltaY, deltaX), true);
                double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);

                //Calculating distance to point
                double hypot = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

                //Calculating velocities
                double angVel = angPid.getPid(pose.rotation, targetRotation);
                double linVel = linPid.getPid(hypot);
                double leftVel = slew(linVel + angVel, prevLeftVel, MAX_RPMPS / 1000, 10);
                double rightVel = slew(linVel - angVel, prevRightVel, MAX_RPMPS / 1000, 10);

                prevLeftVel = leftVel;
                prevRightVel = rightVel;

                //Converting Velocities to voltage
                double leftVoltage = leftVCon.rpmVelToVoltage(leftFrontMotor.get_actual_velocity(), leftVel);
                double rightVoltage = notLeftVCon.rpmVelToVoltage(rightFrontMotor.get_actual_velocity(), rightVel);

                this->setVoltage(leftVoltage, rightVoltage);

                //Exit Conditions
                //Only exit when the average error, and average velcoty of the error of the 2 pids is low, which means its stopped
                //Both these pids are just creating rpms for the wheels, so their magnitude should be really similar, 
                double avgError = (fabs(linPid.getError()) + fabs(angPid.getError())) / 2;
                double avgVelError = (fabs(linPid.getDervative()) + fabs(angPid.getDervative())) / 0.01 / 2;
                if(avgError < errorExit)
                {
                    if(avgVelError < velExit)
                        break; //Comment out for tuning
                }

                pros::delay(10);
                }
            }
        }












/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int intakeMotorPort)
        :   intakeMotor(pros::Motor (intakeMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees))
        {}

        //Function to set intake voltage
        void intake::setVoltage(double voltage)
        {
            intakeMotor.move_voltage(floor(voltage));
        }

        //Function to run intake during driver control
        void intake::driverFunctions()
        {
            setVoltage((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2)) * 12000);
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Plunger Class
        //Constructor
        plunger::plunger(int plungerMotorPort, char armpistonPort, char clampPistonPort)
        :   plungerMotor(pros::Motor (plungerMotorPort, pros::v5::MotorGearset::red, pros::v5::MotorEncoderUnits::degrees)),
            armPiston(pros::adi::Pneumatics(armpistonPort, false, false)),
            clampPiston(pros::adi::Pneumatics(clampPistonPort, true, false))
        {}

        //Function to set plunger voltage
        void plunger::setVoltage(double voltage)
        {
            plungerMotor.move_voltage(floor(voltage));
        }
        void plunger::setArmState(bool state)
        {
            armPiston.set_value(state);
        }
        void plunger::setClampState(bool state)
        {
            clampPiston.set_value(state);
        }

        //Function to run plunger during driver control
        void plunger::driverFunctions()
        {
            setVoltage((Controller.get_digital(DIGITAL_L1) - Controller.get_digital(DIGITAL_L2)) * 12000);

            armPressCount += Controller.get_digital_new_press(DIGITAL_DOWN);
            clampPressCount += Controller.get_digital_new_press(DIGITAL_LEFT); //JOUZAA!!!!!!
            armPressCount % 2 == 0 ? setArmState(false) : setArmState(true);
            clampPressCount % 2 == 0 ? setClampState(true) : setClampState(false);

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
            mogoPressCount += Controller.get_digital_new_press(DIGITAL_A);
            mogoPressCount % 2 == 0 ? setState(false) : setState(true);
        }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}