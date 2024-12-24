#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/globals.h"

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
            setVoltage(((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2))) * 12000);
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Lift Class
        //Constructor
        lift::lift(int liftMotor1Port, int liftMotor2Port)
        :   liftMotor1(pros::Motor (liftMotor1Port, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees)),
            liftMotor2(pros::Motor (liftMotor2Port, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees))
        {}

        //Function to set intake voltage
        void lift::setVoltage(double voltage)
        {
            liftMotor1.move_voltage(floor(voltage));
            liftMotor2.move_voltage(floor(voltage));
        }

        //Function to run intake during driver control
        void lift::driverFunctions()
        {  
            setVoltage(((Controller.get_digital(DIGITAL_L1) - Controller.get_digital(DIGITAL_L2))) * 12000);
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
            pressCount += Controller.get_digital_new_press(MOGO_CONTROL);
            pressCount % 2 == 0 ? setState(true) : setState(false); //End autons with a mogo
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //Rush mech class
        //Constructor
        rushMech::rushMech(char rushSolanoidPort)
        :   rushSolanoid(pros::adi::Pneumatics (rushSolanoidPort, false, false))
        {}

        //Function to set mogo output
        void rushMech::setState(bool state)
        {
            rushSolanoid.set_value(state);
        }

        //Function to run mogo during driver control
        void rushMech::driverFunctions()
        {
            pressCount += Controller.get_digital_new_press(RUSH_CONTROL);
            pressCount % 2 == 0 ? setState(false) : setState(true);
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //Doinker class
        //Constructor
        doinker::doinker(char doinkerSolanoidPort)
        :   doinkerSolanoid(pros::adi::Pneumatics (doinkerSolanoidPort, false, false))
        {}

        //Function to set mogo output
        void doinker::setState(bool state)
        {
            doinkerSolanoid.set_value(state);
        }

        //Function to run mogo during driver control
        void doinker::driverFunctions()
        {
            pressCount += Controller.get_digital_new_press(DOINKER_CONTROL);
            pressCount % 2 == 0 ? setState(false) : setState(true);
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //Pto class
        //Constructor
        pto::pto(char ptoSolanoidPort)
        :   ptoSolanoid(pros::adi::Pneumatics (ptoSolanoidPort, false, false))
        {}

        //Function to set mogo output
        void pto::setState(bool state)
        {
            ptoSolanoid.set_value(state);
        }

        //Function to run mogo during driver control
        void pto::driverFunctions()
        {
            pressCount += Controller.get_digital_new_press(PTO_CONTROL);
            pressCount % 2 == 0 ? setState(false) : setState(true);
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}