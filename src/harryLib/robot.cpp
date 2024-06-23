#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"

//File for controlling all systems in the robot

namespace drivetrain
{
        //constructor
        drivetrain::drivetrain(int leftFrontMotorPort, int leftMidMotorPort, int leftBackMotorPort, int rightFrontMotorPort, int rightMidMotorPort, int rightBackMotorPort)
        :   leftFrontMotor(pros::Motor (leftBackMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)), 
            leftMidMotor(pros::Motor (leftMidMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            leftBackMotor(pros::Motor (leftBackMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            rightFrontMotor(pros::Motor (rightFrontMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            rightMidMotor(pros::Motor (rightMidMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            rightBackMotor(pros::Motor (rightBackMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees))
        {}

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

            int leftOutput = floor(linearToCubed(leftJoystick, 127, 1));
            int rightOutput = floor(linearToCubed(rightJoystick, 127, 1));

            setVoltage(leftOutput, rightOutput);
        }

    //Pose struct

        Pose::Pose(double x, double y, double heading)
        {
            this->x = x;
            this->y = y;
            this->heading = heading;
        }

        void Pose::set(double x, double y, double heading)
        {
            this->x = x;
            this->y = y;
            this->heading = heading;
        }
        void Pose::set(Pose pose)
        {
            this->x = pose.x;
            this->y = pose.y;
        }

    double distanceTravelled;
    bool moving;

    void moveToPose()
    {
        
    }

}

namespace subsystems
{
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

    //Plunger Class
        //Constructor
        plunger::plunger(int plungerMotorPort)
        :   plungerMotor(pros::Motor (plungerMotorPort, pros::v5::MotorGearset::red, pros::v5::MotorEncoderUnits::degrees))
        {}

        //Function to set plunger voltage
        void plunger::setVoltage(double voltage)
        {
            plungerMotor.move_voltage(floor(voltage));
        }

        //Function to run plunger during driver control
        void plunger::driverFunctions()
        {
            setVoltage((Controller.get_digital(DIGITAL_L1) - Controller.get_digital(DIGITAL_L2)) * 12000);
        }

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
}