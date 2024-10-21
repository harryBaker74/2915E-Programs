#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"

namespace subsystems
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int intakeMotorPort, int opticalSensorPort)
        :   intakeMotor(pros::Motor (intakeMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            optical(pros::v5::Optical (opticalSensorPort))
        {}

        //Function to set intake voltage
        void intake::setVoltage(double voltage)
        {
            intakeMotor.move_voltage(floor(voltage));
        }

        //Function to run intake during driver control
        void intake::driverFunctions(bool colour)
        {  
            setVoltage((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2)) * 12000);
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Lift Class
        //Constructor
        lift::lift(int liftMotorPort)
        :   liftMotor(pros::Motor (liftMotorPort, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees))
        {
            liftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
        }

        //Function to set basket voltage
        void lift::setVoltage(double voltage)
        {
            liftMotor.move_voltage(floor(voltage));
        }

        void lift::setPosition(enum LiftPosition position)
        {
            this->targetPos = position;
        }

        void lift::holdPosition(enum LiftPosition position)
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
            double Ks = 700;
            double Kv = 25;
            double Kg = 1200;

            while(this->hold)
            {
                double currentPos = this->liftMotor.get_position() / 5;
                double posError = (targetPos / 5) - currentPos;
                double angle = encoderToRad(currentPos);
                double targetVel = posPID.getPid(posError);
                double currentVel = this->liftMotor.get_actual_velocity();

                double voltage = Kg * cos(angle) + Ks * sign(targetVel) + Kv * targetVel;

                setVoltage(voltage);
                printf("%f\n", voltage);
                pros::delay(15);
            }

            //Setting voltage back to zero so the lift stops trying to hold its position
            setVoltage(0);
            //Kill task when not being used
            pros::Task::current().remove();
            }};
        }

        void lift::endHold()
        {
            this->hold = false;
        }

        double lift::encoderToRad(double encoder)
        {
            double zero = 100 / 5;//Amount of encoder units for a perfectly horizontal position, zero radians
            double offset = zero - encoder; //Find difference, sign doesnt matter because its being used for cos wave
            return offset * M_PI / 180; //Convert from normal encoder unit(degrees) to radians
        }

        //Function to run basket during driver control
        void lift::driverFunctions()
        {
            if(Controller.get_digital(DIGITAL_L1))
                setPosition(subsystems::LiftPosition::SCORE);
            if(Controller.get_digital(DIGITAL_L2))
                setPosition(subsystems::LiftPosition::GRAB);
            if(Controller.get_digital(DIGITAL_LEFT))
                setPosition(subsystems::LiftPosition::DEFAULT);
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
            mogoPressCount % 2 == 0 ? setState(false) : setState(true); //End autons with a mogo
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Arms class
        //Constructer
        arms::arms(char arm1SolanoidPort)
        : arm1Solanoid(pros::adi::Pneumatics (arm1SolanoidPort, false, false))
        {}

        //Function to set arm state
        void arms::setState(bool state)
        {
            arm1Solanoid.set_value(state);
        }

        //Function for driver control
        void arms::driverFunctions()
        {
            arm1PressCount += Controller.get_digital_new_press(DIGITAL_UP);
            arm1PressCount % 2 == 0 ? setState(false) : setState(true);
        }
}