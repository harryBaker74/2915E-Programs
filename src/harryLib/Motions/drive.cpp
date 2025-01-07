#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/odom.hpp"
#include "harryLibHeader/globals.h"
#include "harryLibHeader/exitConditions.hpp"
#include "harryLibHeader/util.hpp"
#include "harryLibHeader/gainSchedular.hpp"
#include "main.h"

namespace subsystems
{
    //A REAL 1D CONTROLLER!!!??!
    void drivetrain::drive(double distance, double maxVoltage, bool async)
    {
        //Prevent multiple motions from running at once
        while (inMotion)
            pros::delay(10);

        

        if (async)
        {
            pros::Task task {[=, this]{
                drive(distance, maxVoltage, false);
                pros::Task::current().remove();
            }};
        }
        else
        {
            this->distanceTraveled = 0;
            this->inMotion = true;


            //Tuned Values
            //10 cm, 40 kp, 390 kd
            //20 cm, 30 kp, 350 kp
            //30 cm, 25 kp, 340 kd
            //45 cm, 22 kp, 260 kd
            //60 cm, 22 kp, 240 kd
            //80 cm, 22 kp, 240 kd

            //Calculations for gain schedular, use graph for better p & k
            //Kp, i = 40, f = 22, p = 4.4, k = 20
            //Kd, i = 390, f = 240, p = 4.8, k = 29

            //Initializing gain schedulers
            gainSchedular Kp = gainSchedular(40, 22, 4.4, 20);
            gainSchedular Kd = gainSchedular(390, 240, 4.8, 29);

            //Tuning
            PID::PID pid = PID::PID(
                Kp.getGain(distance),  //Kp
                0,  //Ki
                Kd.getGain(distance),  //Kd
                0,  //Windup Range
                0   //Max Integral
            );

            double headingKp = 10000;
            
            //Exit Conditions
            double errorExit = 50;
            double velExit = 30;

            //Static variables for derivatives
            Pose prevPose = this->pose;

            //Calculating Target Encoder amounts
            double encoderDistance = (distance / (DRIVE_WHEEL_DIAMETER * 2.54 * M_PI) * 360 / (DRIVE_GEAR_RATIO));
            double targetLeftEncoder =  leftFrontMotor.get_position() + encoderDistance;
            double targetRightEncoder =  rightFrontMotor.get_position() + encoderDistance;

            Controller.print(0, 0, "%f", encoderDistance);

            double startLeftEncoder = Odometery::getEncoder(LEFT_MOTOR_FRONT);//For poistion tracking
            
            //Measuring heading for correction
            double startRotation = pose.rotation;

            int counter = 0;
            while(true)
            {   
                //Getting the voltage output based on the average distance fromt the desired encoder amount
                double output = pid.getPid(
                ((targetLeftEncoder - leftFrontMotor.get_position()) + (targetRightEncoder - rightFrontMotor.get_position())) / 2);
                
                double headingDiff = startRotation - pose.rotation;
                double headingOutput = headingDiff * headingKp;
                                
                Controller.print(0, 0, "%f", output);

                if(fabs(output) > maxVoltage)
                {
                    output = sign(output) * maxVoltage;
                }

                double leftVoltage = output + headingOutput;
                double rightVoltage = output - headingOutput;

                double ratio = fmax(fabs(leftVoltage), fabs(rightVoltage)) / 12000;
                if(ratio > 1)
                {
                    leftVoltage /= ratio;
                    rightVoltage /= ratio;
                }

                //Outputting voltage into motors and slewing
                this->setVoltage(leftVoltage, rightVoltage);

                Controller.print(0, 0, "%.2f, %.2f", output, pid.getError());

                //Updating async variables
                this->distanceTraveled = (Odometery::getEncoder(LEFT_MOTOR_FRONT) - startLeftEncoder) * DRIVE_GEAR_RATIO / 360 * (DRIVE_WHEEL_DIAMETER * 2.54 * M_PI);

                //Exit conditions
                //Getting Current Motor Velocity
                double currentVel = (leftFrontMotor.get_actual_velocity() + rightFrontMotor.get_actual_velocity()) / 2;

                //Checking if error and velocity are within a certain threshold
                if(exitConditions::rangeExit(pid.getError(), errorExit) && exitConditions::rangeExit(currentVel, velExit))
                {
                    break;
                }

                pros::delay(10);
            }
            this->inMotion = false;
        }
    }
}