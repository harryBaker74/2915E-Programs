#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"

namespace subsystems
{

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

    //Basket Class
        //Constructor
        basket::basket(int basketMotorPort, char basketPistonsPort)
        :   basketMotor(pros::Motor (basketMotorPort, pros::v5::MotorGearset::red, pros::v5::MotorEncoderUnits::degrees)),
            basketPistons(pros::adi::Pneumatics (basketPistonsPort, false, false))
        {
            basketMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
        }

        //Function to set basket voltage
        void basket::setVoltage(double voltage)
        {
            basketMotor.move_voltage(floor(voltage));
        }

        void basket::setState(bool state)
        {
            basketPistons.set_value(state);
        }

        void basket::setPosition(enum LiftPosition position)
        {
            this->targetPos = position;
        }

        void basket::holdPosition(enum LiftPosition position)
        {
            this->targetPos = position;
            this->hold = true;

            pros::Task task{[=, this] {
            //Input position, output a velocity
            PID::PID posPID = PID::PID
            {
                10,
                0,
                0,
                0,
                0
            };

            //Inputs a velocity, outputs a voltage
            double Ks = 1000;
            double Kv = 40;
            double Kg = 3800;

            while(this->hold)
            {
                double currentPos = this->basketMotor.get_position() / 3;
                double posError = (targetPos / 3) - currentPos;
                double angle = encoderToRad(currentPos);
                double targetVel = posPID.getPid(posError);
                double currentVel = this->basketMotor.get_actual_velocity();

                double voltage = Kg * cos(angle) + Ks * sign(targetVel) + Kv * targetVel;

                setVoltage(voltage);
                pros::delay(15);
            }

            //Setting voltage back to zero so the lift stops trying to hold its position
            setVoltage(0);
            //Kill task when not being used
            pros::Task::current().remove();
            }};
        }

        void basket::endHold()
        {
            this->hold = false;
        }

        double basket::encoderToRad(double encoder)
        {
            double zero = 200 / 3;//Amount of encoder units for a perfectly horizontal position, zero radians
            double offset = zero - encoder; //Find difference, sign doesnt matter because its being used for cos wave
            return offset * M_PI / 180; //Convert from normal encoder unit(degrees) to radians
        }

        //Function to run basket during driver control
        void basket::driverFunctions()
        {
            basketPressCount += Controller.get_digital_new_press(DIGITAL_LEFT);
            basketPressCount % 2 == 0 ? setState(false) : setState(true);
            this->setVoltage((Controller.get_digital(DIGITAL_L1) - Controller.get_digital(DIGITAL_L2)) * 12000);
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
            mogoPressCount % 2 == 0 ? setState(true) : setState(false); //End autons with a mogo
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}