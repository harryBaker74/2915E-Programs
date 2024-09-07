#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/odom.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/boomerang.hpp"

#include "harryLibHeader/exitConditions.hpp"

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
         * Does not need to be called as a task, it already has lambda task in it.
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


        void drivetrain::setVoltage(double left, double right, bool doSlew, double timestep)
        {      
            //COnverting the double inputs to ints
            int leftVoltage = floor(left);
            int rightVoltage = floor(right);
            //Slewing voltage if specified
            if(doSlew)
            {
                leftVoltage = floor(slew(left, prevLeftVoltage, voltageSlew, timestep));
                rightVoltage = floor(slew(right, prevRightVoltage, voltageSlew, timestep));
            }

            //Setting Left Motors
            leftFrontMotor.move_voltage(leftVoltage);
            leftMidMotor.move_voltage(leftVoltage);
            leftBackMotor.move_voltage(leftVoltage);

            //Setting Right Motors
            rightFrontMotor.move_voltage(rightVoltage);
            rightMidMotor.move_voltage(rightVoltage);
            rightBackMotor.move_voltage(rightVoltage);

            //Updating static variables
            prevLeftVoltage = leftVoltage;
            prevRightVoltage = rightVoltage;
        }

        /**
         * @brief Function that handles controller inputs for drivetrain. Intended to be used during Op Control
         * 
         */
        void drivetrain::driverFunctions()
        {
            int leftJoystick = Controller.get_analog(ANALOG_LEFT_Y);
            int rightJoystick = Controller.get_analog(ANALOG_RIGHT_Y);

            int leftOutput = lineartoSquared(leftJoystick, 127, 1);
            int rightOutput = lineartoSquared(rightJoystick, 127, 1);

            this->setVoltage(leftOutput * 12000/127, rightOutput * 12000/127);
        }

        void drivetrain::waitUntil(double distance)
        {
            while ((this->distanceTraveled < distance) && (this->inMotion))
                pros::delay(10);
        }

        void drivetrain::waitUntilEnd()
        {
            while(this->inMotion)
                pros::delay(10);
        }


    void drivetrain::stop(int timeMs)
    {
        int endTime = pros::millis() + timeMs;
        this->setVoltage(0, 0);
        while(pros::millis() < endTime)
            pros::delay(10);

    }

    

    void drivetrain::turnToHeading(double heading, int timeout_ms, bool radians, bool async)
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
        int endTime = pros::millis() + timeout_ms;
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
                13000.0,    //Kp
                2000.0,    //Ki
                100000.0,    //Kd
                0.05,    //Windup Range
                0.0     //Max Intergal
            );

            //Exit conditions
            double errorExit = 0.01;
            double velExit = 0.005;

            
        //Everything else
        double angle = heading;
        double prevVel = 0;
        int counter = 0;
        
        if(!radians) angle *= M_PI / 180;  //Converting to radians if needed

        //Finding the target rotation
        //Gets the difference between the desired bounded heading and the current bounded heading, then bounds this angle, then gets the current rotation added to it
        //In a roundabout way because we dont want to do turns bigger than 180 deg/1 pi, and its easier to track because the values cant overflow due to using rotation
        double targetRotation = pose.rotation + (boundAngle(angle - pose.heading, true));

        while(pros::millis() < endTime)
        {
            //Pid Voltage Calculations
            double output = pid.getPid(pose.rotation, targetRotation);
            
            Controller.print(0, 0, "%f", output);


            //Setting drivetrain voltage based on Pid Output and Slewing
            this->setVoltage(output, -output, true, 10);

            //Exit Conditions
            //Error, and Velocity based(only exit when both are low) so we dont need to worry about waiting with low error for a certain amount of time
            //Because error is in rad: vel is rad/s
            double error = pid.getError();
            double errorVel = pid.getDervative() / 0.01; //Derivative is already just the rate of change

            //Checking if all these values are below a certain threshold
            if (exitConditions::rangeExit(error, errorExit) && exitConditions::rangeExit(errorVel, velExit))
                break;

            //Updating distance traveled for async functions
            this->distanceTraveled += this->pose.rotation - this->prevPose.rotation;
            
            //Delay for scheduling
            pros::delay(10);
        }
        this->inMotion = false;
    };
}
}