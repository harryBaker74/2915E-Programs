#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"
#include "../include/harryLibHeader/odom.hpp"
#include "../include/harryLibHeader/pid.hpp"
#include"../include/harryLibHeader/velocityController.hpp"

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
        //AUTON FUNCTIONS

        void drivetrain::turnToHeading(double heading, bool radians)
        {
            pros::Task task {[=, this] {

                PID::PID pid(
                    0.0,    //Kp
                    0.0,    //Ki
                    0.0,    //Kd
                    0.0,    //Windup Range
                    0.0     //Max Intergal
                );

                vController::vController vCon;

                double angle = heading;
                double prevOutput; //For slew
                
                if(!radians) angle *= M_PI / 180;  //Converting to radians if needed

                double targetRotation = pose.rotation + angle; //Finding the target rotation

                while(true)
                {
                    //Pid Velocity Calculations
                    double output = pid.getPid(pose.rotation, targetRotation);
                    //Giving a slew(rate limiter) to the output, so we dont accel to fast
                    double velocity = slew(output, prevOutput, 0, 10);
                    prevOutput = output;


                    //Getting current motor velocity
                    std::vector <double> leftVels = leftDriveMotors.get_actual_velocity_all();
                    double averageVel = (leftVels.at(0) + leftVels.at(1) + leftVels.at(2)) / 3; //Not divided by 2 becasue the slew is standard
                    //Velocity controller converting desired motor velocity into voltage
                    double voltage = vCon.rpmVelToVoltage(averageVel, velocity);  
                    //Setting drivetrain voltage based on Pid Output, Slew, and Velocity controller output
                    this->setVoltage(voltage, -voltage);

                    //Exit Conditions

                    
                    pros::delay(10);
                }

                pros::Task::current().remove();
            }};
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