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
                        0
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
            //Wait for motion to start
            while(!this->inMotion)
                pros::delay(10); 

            while ((fabs(this->distanceTraveled) < distance) && (this->inMotion))
            {
                pros::delay(10);
            }
        }

        void drivetrain::waitUntilEnd()
        {
            while(this->inMotion)
                pros::delay(10);
        }


        void drivetrain::stop(int timeMs)
        {
            int endTime = pros::millis() + timeMs;

            leftDriveMotors.set_brake_mode(MOTOR_BRAKE_HOLD);
            rightDriveMotors.set_brake_mode(MOTOR_BRAKE_HOLD);

            leftDriveMotors.brake();
            rightDriveMotors.brake();

            while (pros::millis() < endTime)
                pros::delay(10);

            leftDriveMotors.set_brake_mode(MOTOR_BRAKE_COAST);
            rightDriveMotors.set_brake_mode(MOTOR_BRAKE_COAST);

        }

        void drivetrain::setBrakeMode(enum pros::motor_brake_mode_e brakeMode)
        {
            leftDriveMotors.set_brake_mode_all(brakeMode);
            rightDriveMotors.set_brake_mode_all(brakeMode);
        }
}