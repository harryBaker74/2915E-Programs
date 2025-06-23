#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/globals.h"

namespace subsystems
{

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Intake Class
        //Constructor
        intake::intake(int intakeMotorPort, int intakeOpticalPort, char intakeLiftSolanoidPort, int liftMotorPort, int rotationSensorPort, int liftOpticalPort)
        :   intakeMotor(pros::Motor (intakeMotorPort, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees)),
            intakeOptical(pros::Optical (intakeOpticalPort)),
            intakeLift(pros::adi::Pneumatics(intakeLiftSolanoidPort, false, false)),
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

        void intake::setIntakeLiftState(bool state)
        {
            intakeLift.set_value(state);
        }

        void intake::setLiftVoltage(double voltage)
        {
            liftMotor.move_voltage(floor(voltage));
        }

        void intake::setRingSortColour(bool colour)
        {
            sortColour = colour;
        }

        void intake::setColourSortState(bool state)
        {
            colourSortingOn = state;
        }

        void intake::waitForRing(int timeout_ms)
        {
            int endTime = pros::millis() + timeout_ms;

            while((!allianceRing) && (pros::millis() <= endTime))
            {
                pros::delay(10);
            }
        }

        void intake::setLoadStartingPosition()
        {
            startingPosition = double(LOAD);
            liftPressCount = 0;
        }

        void intake::holdPosition(LiftPosition pos)
        {

            targetPos = pos;
            if(!holding)
            {
                liftMotor.tare_position();
                rotationSensor.reset_position();

                pros::Task task{[=, this] {
                    holding = true;

                    PID::PID posPID = PID::PID(
                        100,
                        0,
                        100,
                        0,
                        0
                    );

                    double Kv = 1;
                    double Ks = 0;
                    double Kg = 0;

                    while(true)
                    {
                        // double currentPos = startingPosition + liftMotor.get_position();
                        double currentPos = startingPosition + (rotationSensor.get_position() / 100);
                        double posError = (2 * double(targetPos)) - currentPos;
                        double angle = fabs(sin(((currentPos - double(LiftPosition::ZERO)) * M_PI / 180)));
                        
                        // if ((posError <= 10) && (currentPos == double(DOUBLERING)))
                        //     posPID.setKp(150);
                        // else
                        //     posPID.setKp(350);

                        if(currentPos <= 0)
                            rotationSensor.reset_position();
                        // else if((currentPos <= 80) && (currentPos >= 5) && (targetPos == DEFAULT) && (liftMotor.get_efficiency() <= 1) && (posError < 0))
                        //     rotationSensor.reset_position();

                        double targetVel = posPID.getPid(posError);

                        Controller.print(0, 0, "%.2f, %.2f",currentPos, double(targetPos));

                        if(Controller.get_digital_new_press(DIGITAL_UP))
                        {
                            startingPosition = 0;
                            rotationSensor.reset_position();
                        }

                        double voltage = (Ks * sign(targetVel)) + (Kg * angle) + (Kv * targetVel);
                        setLiftVoltage(voltage);

                        pros::delay(20);
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
                        //Reversing intake to slow it down if currently sorting and colour sorting is enabled
                        setIntakeVoltage(-3000);
                }
                //If we shouldnt currently be sorting
                else
                {
                    if(colourSortingOn)
                    {
                        //Detecting ring
                        double current = intakeOptical.get_hue();
                        //Ring to sort
                        if((sortColour ? ((blueMin < current) && (blueMax > current)) : ((redMin < current) && (redMax > current))) && (intakeOptical.get_proximity() > 180))
                        {
                            //Setting sort start pos if ring is detected
                            sortStartTime = pros::millis() + 50;
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
                        liftPressCount = 0;    
                    }
                    else if(wall || alliance)
                    {
                        alliance = false;
                        tip = false;
                        wall = false;
                        liftPressCount = 1;
                    }
                    else
                    {
                        liftPressCount += 1;
                    }

                    
                    if(liftPressCount == 3)
                    liftPressCount = 0;
                
                    if(!alliance && !tip && !wall)
                    {
                        if(liftPressCount == 0)
                            holdPosition(DEFAULT);
                        else if(liftPressCount == 1)
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
                    liftPressCount = 0;
                    holdPosition(ALLIANCE);
                }
                if(Controller.get_digital_new_press(DIGITAL_A))
                {
                    tip = true;
                    alliance = false;
                    wall = false;
                    liftPressCount = 0;
                    holdPosition(TIP);
                }
                if(Controller.get_digital_new_press(DIGITAL_RIGHT))
                {
                    wall = true;
                    alliance = false;
                    tip = false;
                    liftPressCount = 0;
                    holdPosition(WALL);
                }

                //Extra control
                if(liftPressCount == 1)
                {
                    //Lowering lift for colour sorting, raising back up if not colour sorting
                    if(sorting)
                        holdPosition(DEFAULT);
                    else if(pros::millis() >= sortEndTime)
                        holdPosition(LOAD);
                }
                
                //Hold alliance ring when ring in lb
                if(allianceRing && (liftOptical.get_proximity() > 200) && ((liftPressCount == 2) || wall))
                {
                    setIntakeVoltage(-500);
                }

                //Turn colour sorting on / off
                colourSortingPressCount += Controller.get_digital_new_press(DIGITAL_X);
                colourSortingOn = (colourSortingPressCount % 2 == 0);
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
                                sortStartTime = pros::millis() + 50;
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
                            setIntakeVoltage(autonVoltage);
                        }

                        //Anti Jam
                        if((fabs(intakeMotor.get_voltage()) >= 1000) && (intakeMotor.get_efficiency() <= 5))
                            setIntakeVoltage(-12000);

                        pros::delay(10);
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
    //Doinkers class
        //Constructor
        doinkers::doinkers(char leftDoinkerSolanoidPort, char rightDoinkerSolanoidPort)
        :   leftDoinker(pros::adi::Pneumatics(leftDoinkerSolanoidPort, false, false)),
            rightDoinker(pros::adi::Pneumatics(rightDoinkerSolanoidPort, false, false))
        {}

        void doinkers::setStates(bool left, bool right)
        {
            leftDoinker.set_value(right);
            rightDoinker.set_value(left);
        }

        void doinkers::driverFunctions()
        {
            leftPressCount += Controller.get_digital_new_press(LEFT_DOINKER_CONTROL);
            rightPressCount += Controller.get_digital_new_press(RIGHT_DOINKER_CONTROL);

            bool leftState = (leftPressCount % 2 != 0);
            bool rightState = (rightPressCount % 2 != 0);

            setStates(rightState, leftState);
        }   

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Goal grabber class
        //Constructor
        goalGrabber::goalGrabber(char goalGrabberSolanoidPort)
        :   goalGrabberSolanoid(pros::adi::Pneumatics(goalGrabberSolanoidPort, false))
        {}

        void goalGrabber::setState(bool state)
        {
            goalGrabberSolanoid.set_value(state);
        }

        void goalGrabber::driverFunctions()
        {
            //Intake lift
            pressCount += Controller.get_digital_new_press(GOAL_GRABBER_CONTROL);
            setState(pressCount % 2 != 0);
        }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}