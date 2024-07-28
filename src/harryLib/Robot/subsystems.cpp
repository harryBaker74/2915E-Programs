#include "harryLibHeader/robot.hpp"

namespace subsystems
{

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
            currentPosition = basketLeftMotor.get_position();

            double slowdown = (prevPosition - currentPosition) * Kd;

            this->setVoltage(((Controller.get_digital(DIGITAL_L1) - Controller.get_digital(DIGITAL_L2)) * speed ) + Kg + slowdown);

            prevPosition = currentPosition;
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