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

        void intake::waitForRing()
        {
            double current = opticalSensor.get_hue();
            while((!sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (opticalSensor.get_proximity() > 180))
            {
                current = opticalSensor.get_hue();
                pros::delay(10);
            }
        }

        //Function to run intake during driver control
        void intake::driverFunctions()
        {  
            //Colour sorting
            //If we should be currently sorting
            /*if(pros::millis() < sortEndTime)
            {
                //Reversing intake to slow it down if currently sorting
                setVoltage(-3000);
            }
            //If we shouldnt currently be sorting
            else
            {
                //Detecting ring
                double current = opticalSensor.get_hue();
                if((sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (opticalSensor.get_proximity() > 180))
                {
                    //Setting sort start pos if ring is detected
                    sortStartTime = pros::millis() + sortStartTimeOffset;
                    sorting = true;
                }

                //Setting the end time for reversing once intake has reached correct position
                if((pros::millis() >= sortStartTime) && sorting)
                {
                    sortEndTime = pros::millis() + sortTime;
                    sorting = false;
                }

                //Normal driver functions for when not sorting
                setVoltage(((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2))) * 12000);
            }
            */
            //Uncomment for when no sorting
            setVoltage(((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2))) * 12000);

            if(Controller.get_digital(DIGITAL_L1))
                setVoltage(-12000);
        }

        void intake::autonFunctions(double voltage)
        {
            autonVoltage = voltage;

            if(!auton)
            {
                auton = true;
                pros::Task task{[=, this]{
                    
                    while(auton)
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
                            if((sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (opticalSensor.get_proximity() > 180))
                            {
                                //Setting sort start pos if ring is detected
                                sortStartTime = pros::millis() + sortStartTimeOffset;
                                sorting = true;
                            }

                            //Setting the end time for reversing once intake has reached correct position
                            if((pros::millis() >= sortStartTime) && sorting)
                            {
                                sortEndTime = pros::millis() + sortTime;
                                sorting = false;
                            }

                            //Normal driver functions for when not sorting
                            setVoltage(autonVoltage);
                        }

                        pros::delay(20);
                    }

                    pros::Task::current().remove();
                }};
            }
        }

        void intake::endAutoTask()
        {
            auton = false;
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Lift Class
        //Constructor
        lift::lift(int liftMotorPort, int rotationSensorPort)
        :   liftMotor(pros::Motor (liftMotorPort, pros::v5::MotorGearset::red, pros::v5::MotorEncoderUnits::degrees)),
            rotationSensor(pros::Rotation (rotationSensorPort))
        {}
        //Function to set intake voltage
        void lift::setVoltage(double voltage)
        {
            liftMotor.move_voltage(floor(voltage));
        }

        void lift::holdPosition(LiftPosition pos)
        {

            targetPos = pos;
            if(!holding)
            {
                rotationSensor.reset_position();

                pros::Task task{[=, this] {
                    holding = true;

                    PID::PID posPID = PID::PID(
                        350,
                        0,
                        500,
                        0,
                        0
                    );

                    double Kv = 1;
                    double Ks = 0;
                    double Kg = 0;


                    while(true)
                    {
                        // double currentPos = rotationSensor.get_position() / 100;
                        double currentPos = liftMotor.get_position();
                        
                        double posError = targetPos - currentPos;
                        double angle = fabs(sin(((currentPos - LiftPosition::ZERO) * M_PI / 180)));
                        
                        // if ((posError <= 10))// && ((currentPos >= DOUBLERING - 10) && (currentPos <= DOUBLERING + 10)))
                            // posPID.setKp(150);
                        // else
                            // posPID.setKp(400);

                        double targetVel = posPID.getPid(posError);

                        double voltage = (Ks * sign(targetVel)) + (Kg * angle) + (Kv * targetVel);
                        setVoltage(voltage);

                        Controller.print(0, 0, "%f", currentPos);

                        if(liftMotor.get_position() <= 0)
                            liftMotor.tare_position();

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
                if(tip)
                {
                    alliance = false;
                    tip = false;
                    wall = false;
                    pressCount = 0;    
                }
                else if(wall || alliance)
                {
                    alliance = false;
                    tip = false;
                    wall = false;
                    pressCount = 1;
                }
                else
                    pressCount += 1;
            }

            if(Controller.get_digital_new_press(DIGITAL_Y))
            {
                alliance = true;
                tip = false;
                wall = false;
                holdPosition(ALLIANCE);
            }
            if(Controller.get_digital_new_press(DIGITAL_A))
            {
                tip = true;
                alliance = false;
                wall = false;
                holdPosition(TIP);
            }
            if(Controller.get_digital(DIGITAL_RIGHT))
            {
                wall = true;
                alliance = false;
                tip = false;
                holdPosition(WALL);
            }

            if(pressCount == 3)
                pressCount = 0;
        
            if(!alliance && !tip && !wall)
            {
                if(pressCount == 0)
                    holdPosition(DEFAULT);
                else if(pressCount == 1)
                    holdPosition(LOAD);
                else
                    holdPosition(DOUBLERING);
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