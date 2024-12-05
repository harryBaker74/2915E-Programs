#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"

namespace subsystems
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int intakeMotor1Port, int intakeMotor2Port)
        :   intakeMotor1(pros::Motor (intakeMotor1Port, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            intakeMotor2(pros::Motor (intakeMotor2Port, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees))
        {}

        double intake::getPosition()
        {
            return (intakeMotor1.get_position() + intakeMotor2.get_position()) / 2;
        }

        double intake::getVelocity()
        {
            return (intakeMotor1.get_actual_velocity() + intakeMotor2.get_actual_velocity()) / 2;
        }

        void intake::setPosition(enum LiftPosition position)
        {
            this->targetPos = position;
        }

        void intake::holdPosition(enum LiftPosition position)
        {
            intakeMotor1.tare_position();
            intakeMotor2.tare_position();

            this->targetPos = position;
            this->lifting = true;

            pros::Task task{[=, this] {
            //Input position, output a velocity
            PID::PID posPID = PID::PID
            {
                6,
                0,
                0,
                0,
                0
            };

            //Inputs a velocity, outputs a voltage
            double Ks = 1000;
            double Kv = 10;
            double Kg = -3000;

            while(this->lifting)
            {

                double currentPos = getPosition();
                double posError = (targetPos) - currentPos;
                double angle = encoderToRad(currentPos);
                double targetVel = posPID.getPid(posError);
                double currentVel = getVelocity();

                double voltage = Kg + Ks * sign(targetVel) + Kv * targetVel;

                Controller.print(0, 0, "%f", targetVel);

                this->setVoltage(voltage);
                pros::delay(15);
            }

            //Setting voltage back to zero so the lift stops trying to hold its position
            setVoltage(0);
            //Kill task when not being used
            pros::Task::current().remove();
            }};
        }

        void intake::endHold()
        {
            this->lifting = false;
        }

        double intake::encoderToRad(double encoder)
        {
            double zero = -30;//Amount of encoder units for a perfectly horizontal position, zero radians
            double offset = zero - encoder; //Find difference, sign doesnt matter because its being used for cos wave
            return offset * M_PI / 180; //Convert from normal encoder unit(degrees) to radians
        }

        //Function to set intake voltage
        void intake::setVoltage(double voltage)
        {
            intakeMotor1.move_voltage(floor(voltage));
            intakeMotor2.move_voltage(floor(voltage));
        }

        //Function to run intake during driver control
        void intake::driverFunctions(bool colour)
        {  
            if(Controller.get_digital(DIGITAL_R1))
            {
                    endHold();
            }
            if(!lifting)
            {
                setVoltage((Controller.get_digital(DIGITAL_R1)) * 12000);
            }

            if(Controller.get_digital(DIGITAL_R2))
            {
                if(!lifting)
                {
                    lifting = true;
                    holdPosition(WALL);
                }
                else
                    setPosition(WALL);

            }
            if(Controller.get_digital(DIGITAL_L2))
            {
                if(!lifting)
                {
                    lifting = true;
                    holdPosition(ALLIANCE);
                }
                else
                    setPosition(ALLIANCE);
            }
            if(Controller.get_digital(DIGITAL_X))
            {
                if(!lifting)
                {
                    lifting = true;
                    holdPosition(TIP);
                }
                else
                    setPosition(TIP);
            }

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
            mogoPressCount % 2 == 0 ? setState(true) : setState(false); //End autons with a mogo
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Redirect Class
        //Constructor
        redirect::redirect(char redirectPistonPort)
        :   redirectPiston(pros::adi::Pneumatics (redirectPistonPort, false))
        {}

        //Function to set redrect output
        void redirect::setState(bool state)
        {
            redirectPiston.set_value(state);
        }

        //Function for driver control
        void redirect::driverFunctions()
        {
            redirectPressCount += Controller.get_digital_new_press(DIGITAL_UP);
            redirectPressCount % 2 == 0 ? setState(false) : setState(true);
        }
}