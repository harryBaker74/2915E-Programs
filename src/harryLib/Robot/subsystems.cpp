#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/globals.h"

namespace subsystems
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int intakeMotorPort, int intakeOpticalPort, int liftMotorPort, int rotationSensorPort, int liftOpticalPort)
        :   intakeMotor(pros::Motor (intakeMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            intakeOptical(pros::Optical (intakeOpticalPort)),
            liftMotor(pros::Motor (liftMotorPort, pros::v5::MotorGearset::red, pros::v5::MotorEncoderUnits::degrees)),
            rotationSensor(pros::Rotation (rotationSensorPort)),
            liftOptical(pros::Optical (liftOpticalPort))
        {
            intakeOptical.set_integration_time(5);
            intakeOptical.set_led_pwm(100);

            liftOptical.set_integration_time(5);
            liftOptical.set_led_pwm(100);
        }

        //Function to set intake voltage
        void intake::setIntakeVoltage(double voltage)
        {   
            intakeMotor.move_voltage(floor(voltage));
        }

        void intake::setLiftVoltage(double voltage)
        {
            liftMotor.move_voltage(floor(voltage));
        }

        void intake::setRingSortColour(bool colour)
        {
            sortColour = colour;
        }

        void intake::waitForRing()
        {
            double current = intakeOptical.get_hue();
            while((!sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (intakeOptical.get_proximity() > 180))
            {
                current = intakeOptical.get_hue();
                pros::delay(10);
            }
        }



        void intake::holdPosition(LiftPosition pos)
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
                        double currentPos = rotationSensor.get_position() / 100;
                        
                        double posError = double(targetPos) - currentPos;
                        double angle = fabs(sin(((currentPos - double(LiftPosition::ZERO)) * M_PI / 180)));
                        
                        if ((posError <= 10) && (currentPos == DOUBLERING))
                            posPID.setKp(150);
                        else
                            posPID.setKp(350);

                        double targetVel = posPID.getPid(posError);

                        double voltage = (Ks * sign(targetVel)) + (Kg * angle) + (Kv * targetVel);
                        setLiftVoltage(voltage);

                        pros::delay(10);
                    }
                }};
            }
        }



        //Function to run intake during driver control
        void intake::driverFunctions()
        {  
            //Intake Control
                //Colour sorting
                //If we should be currently sorting
                if(pros::millis() < sortEndTime)
                {
                    //Reversing intake to slow it down if currently sorting
                    setIntakeVoltage(-3000);
                }
                //If we shouldnt currently be sorting
                else
                {
                    //Detecting ring
                    double current = intakeOptical.get_hue();
                    //Ring to sort
                    if((sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (intakeOptical.get_proximity() > 180))
                    {
                        //Setting sort start pos if ring is detected
                        sortStartTime = pros::millis() + sortStartTimeOffset;
                        sorting = true;
                        allianceRing = false;
                    }
                    //Ring to not sort
                    else if((!sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (intakeOptical.get_proximity() > 180))
                    {
                        allianceRing = true;
                    }
                    else
                        allianceRing = false;

                    //Setting the end time for reversing once intake has reached correct position
                    if((pros::millis() >= sortStartTime) && sorting)
                    {
                        sortEndTime = pros::millis() + sortTime;
                        sorting = false;
                    }

                    //Normal driver functions for when not sorting
                    setIntakeVoltage(((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2))) * 12000);
                }

                //Uncomment for when no sorting
                // setVoltage(((Controller.get_digital(DIGITAL_R1) - Controller.get_digital(DIGITAL_R2))) * 12000);


            //Lift control
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
                    {
                        pressCount += 1;
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

            //Lift control again
                if(Controller.get_digital_new_press(DIGITAL_Y))
                {
                    alliance = true;
                    tip = false;
                    wall = false;
                    pressCount = 0;
                    holdPosition(ALLIANCE);
                }
                if(Controller.get_digital_new_press(DIGITAL_A))
                {
                    tip = true;
                    alliance = false;
                    wall = false;
                    pressCount = 0;
                    holdPosition(TIP);
                }
                if(Controller.get_digital_new_press(DIGITAL_RIGHT))
                {
                    wall = true;
                    alliance = false;
                    tip = false;
                    pressCount = 0;
                    holdPosition(WALL);
                }

                //Extra control
                if(pressCount == 1)
                {
                    //Reversing intake if lifting to prevent jamming
                    if(Controller.get_digital(DIGITAL_L1) || Controller.get_digital(DIGITAL_RIGHT) || Controller.get_digital(DIGITAL_A) || Controller.get_digital(DIGITAL_Y))
                        setIntakeVoltage(-4000);

                    //Lowering lift for colour sorting, raising back up if not colour sorting
                    if(sorting)
                        holdPosition(DEFAULT);
                    else if(pros::millis() >= sortEndTime)
                        holdPosition(LOAD);
                }
                
                if(allianceRing && (liftOptical.get_proximity() > 200) && ((pressCount == 2) || wall))
                {
                    setIntakeVoltage(-500);
                }
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
                            setIntakeVoltage(-3000);
                        }
                        //If we shouldnt currently be sorting
                        else
                        {
                            //Detecting ring
                            double current = intakeOptical.get_hue();
                            if((sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (intakeOptical.get_proximity() > 180))
                            {
                                //Setting sort start pos if ring is detected
                                sortStartTime = pros::millis() + sortStartTimeOffset;
                                sorting = true;
                                holdPosition(LiftPosition::DEFAULT);
                            }

                            //Setting the end time for reversing once intake has reached correct position
                            if((pros::millis() >= sortStartTime) && sorting)
                            {
                                sortEndTime = pros::millis() + sortTime;
                                sorting = false;
                            }

                            //Normal driver functions for when not sorting
                            setIntakeVoltage(autonVoltage);
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
    //Mogo class
        //Constructor
        mogo::mogo(char mogoSolanoidPort, int mogoOpticalPort)
        :   mogoSolanoid(pros::adi::Pneumatics (mogoSolanoidPort, false, false)),
            opticalSensor(pros::Optical(mogoOpticalPort))
        {
            opticalSensor.set_integration_time(5);
            opticalSensor.set_led_pwm(100);
        }

        //Function to set mogo output
        void mogo::setState(bool state)
        {
            mogoSolanoid.set_value(state);
        }

        //Function to run mogo during driver control
        void mogo::driverFunctions()
        {
            if((opticalSensor.get_proximity() >= 255) && (opticalSensor.get_hue() >= hueMin) && (opticalSensor.get_hue() <= hueMax))
            {
                if(primed && !holding)
                {
                    Controller.rumble("...");
                    setState(true);
                    holding = true;
                    primed = false;
                }
            
            }
            else
            {
                if(opticalSensor.get_proximity() <= 100)
                    primed = true;
            }

            if(Controller.get_digital_new_press(MOGO_CONTROL))
            {
                if(!holding)
                {
                    setState(true);
                    holding = true;
                }
                else
                {
                    setState(false);
                    holding = false;
                    primed = false;
                    startCheckTime = pros::millis() + 300;
                }
            }
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