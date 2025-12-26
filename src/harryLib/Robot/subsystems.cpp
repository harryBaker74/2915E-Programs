#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/globals.h"

namespace subsystems
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int lowerMotor1Port, int lowerMotor2Port, int upperMotorPort, char hoodSolanoidPort)
        :   lowerMotor1(pros::Motor (lowerMotor1Port, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            lowerMotor2(pros::Motor(lowerMotor2Port, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees)),
            upperMotor(pros::Motor (upperMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            hoodSolanoid(pros::adi::Pneumatics (hoodSolanoidPort, false))
        {}

        //Function to set intake voltage
        void intake::setIntakeState(double lowerVoltage, double upperVoltage, bool hoodSolanoidState)
        {   
            lowerMotor1.move_voltage(floor(lowerVoltage));
            lowerMotor2.move_voltage(floor(lowerVoltage));
            upperMotor.move_voltage(floor(upperVoltage));
            hoodSolanoid.set_value(hoodSolanoidState);
        }

        //Function to run intake during driver control
        void intake::driverFunctions()
        {
            //Hood control, toggle
            hoodPressCount += Controller.get_digital_new_press(DIGITAL_Y);

            // Intake Control
            double lowerVoltage = 0;
            double upperVoltage = 0;

            // Intaking from bot to top
            if(Controller.get_digital(DIGITAL_R1))
            {
                lowerVoltage = 12000;
                upperVoltage = 12000;
            }
            // Outtaking from top to bot
            else if(Controller.get_digital(DIGITAL_R2))
            {
                lowerVoltage = -12000;
                upperVoltage = -12000;
            }
            // Redirecting and scoring mid
            else if(Controller.get_digital(DIGITAL_L1))
            {   
                lowerVoltage = 12000;
                upperVoltage = -12000;
            }
            setIntakeState(lowerVoltage, upperVoltage, hoodPressCount % 2 != 0);
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Matchload class
        //Constructor
        matchload::matchload(char matchloadSolanoidPort) 
        :   matchloadSolanoid(pros::adi::Pneumatics (matchloadSolanoidPort, false))
        {}

        void matchload::setState(bool matchloadState)
        {
            matchloadSolanoid.set_value(matchloadState);
        }

        void matchload::driverFunctions()
        {
            matchloadPressCount += Controller.get_digital_new_press(DIGITAL_RIGHT);

            //pressed odd amount of times
            if(matchloadPressCount % 2 != 0)
            {
                setState(1);
            }
            else
            {
                setState(0);
            }  
        }

    //Descore class
        //Constructor
        descore::descore(char descoreSolanoidPort) 
        :   descoreSolanoid(pros::adi::Pneumatics (descoreSolanoidPort, false))
        {}

        void descore::setState(bool state)
        {
            descoreSolanoid.set_value(!state);
        }

        void descore::driverFunctions()
        {
            pressCount += Controller.get_digital_new_press(DIGITAL_L2);
            setState(pressCount % 2 != 0);
        }
}