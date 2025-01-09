#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/globals.h"

namespace subsystems
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int intakeMotorPort, int opticalSensorPort)
        :   intakeMotor(pros::Motor (intakeMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            opticalSensor(pros::Optical (opticalSensorPort))
        {
            opticalSensor.set_integration_time(5);
            opticalSensor.set_led_pwm(100);
        }

        //Function to set intake voltage
        void intake::setVoltage(double voltage)
        {
            intakeMotor.move_voltage(floor(voltage));
        }

        void intake::setRingSortColour(bool colour)
        {
            sortColour = colour;
        }

        //Function to run intake during driver control
        void intake::driverFunctions()
        {  
            //Colour sorting
            //If we should be currently sorting
            if(pros::millis() < sortEndTime)
            {
                //Reversing intake to slow it down if currently sorting
                setVoltage(-3000);
            }
            //If we shouldnt currently be sorting
            else
            {
                //Detecting ring
                double current = opticalSensor.get_hue();
                if((sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (opticalSensor.get_proximity() > 220))
                {
                    //Setting sort start pos if ring is detected
                    sortStartPos = intakeMotor.get_position() + sortStartPosOffset;
                    sortStartCheckTime = pros::millis() + checkDelay;//Delay for sortStartPos to have time to set correctly
                    sorting = true;
                }

                //Setting the end time for reversing once intake has reached correct position
                if((sortStartPos <= intakeMotor.get_position()) && sorting && (pros::millis()>= sortStartCheckTime))
                {
                    sortEndTime = pros::millis() + sortTime;
                    sorting = false;
                }

                //Normal driver functions for when not sorting
                setVoltage(((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2))) * 12000);
            }
            
            //Uncomment for when no sorting
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

        void lift::holdPosition(LiftPosition pos)
        {

            targetPos = pos;
            if(!holding)
            {
                liftMotor1.tare_position();
                liftMotor2.tare_position();

                pros::Task task{[=, this] {
                    
                    holding = true;

                    PID::PID posPID = PID::PID(
                        200,
                        0,
                        100,
                        0,
                        0
                    );

                    double Kv = 1;
                    double Ks = 0;
                    double Kg = -3000;

                    while(true)
                    {
                        double currentPos = (liftMotor1.get_position() + liftMotor2.get_position()) / 2;
                        double posError = targetPos - currentPos;
                        double angle = fabs(sin((currentPos - LiftPosition::ZERO) / 2));
                        double targetVel = posPID.getPid(posError);

                        double voltage = (Ks * sign(targetVel)) + (Kg * angle) + (Kv * targetVel);
                        setVoltage(voltage);

                        if(currentPos <= 0)
                        {
                            liftMotor1.tare_position();
                            liftMotor2.tare_position();
                        }

                        Controller.print(0, 0, "%f", currentPos);

                        pros::delay(10);
                    }
                }};
            }
        }

        //Function to run intake during driver control
        void lift::driverFunctions()
        {  
            if(Controller.get_digital_new_press(DIGITAL_L1))
            {
                alliance = false;
                pressCount += 1;
            }

            if(Controller.get_digital_new_press(DIGITAL_A))
            {
                alliance = true;
                holdPosition(ALLIANCE);
            }
            
            if(pressCount == 3)
            pressCount = 0;
        
            if(!alliance)
            {
                if(pressCount == 0)
                    holdPosition(DEFAULT);
                else if(pressCount == 1)
                    holdPosition(LOAD);
                else if(pressCount == 2)
                    holdPosition(WALL);
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
            pressCount += Controller.get_digital_new_press(MOGO_CONTROL);
            pressCount % 2 == 0 ? setState(false) : setState(true); //End autons with a mogo
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