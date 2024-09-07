#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/odom.hpp"
#include "harryLibHeader/globals.h"
#include "harryLibHeader/exitConditions.hpp"
#include "harryLibHeader/util.hpp"
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
                drive(distance, false);
                pros::Task::current().remove();
            }};
        }
        else
        {
            this->distanceTraveled = 0;
            this->inMotion = true;

            //Tuning
            PID::PID pid = PID::PID(
                25,  //Kp
                0,  //Ki
                5,  //Kd
                0,  //Windup Range
                0   //Max Integral
            );

            double headingKp = 400;
            
            //Exit Conditions
            double errorExit = 120;
            double velExit = 20;

            //Static variables for derivatives
            Pose prevPose = this->pose;

            //Calculating Target Encoder amounts
            double encoderDistance = (distance / (DRIVE_WHEEL_DIAMETER * M_PI) * 360 * (DRIVE_GEAR_RATIO));
            double targetLeftEncoder = Odometery::getEncoder(LEFT_MOTOR_FRONT) + encoderDistance;
            double targetRightEncoder =  Odometery::getEncoder(RIGHT_MOTOR_FRONT) + encoderDistance;
            
            //Measuring heading for correction
            double startRotation = pose.rotation;
            while(true)
            {   
                //Getting the voltage output based on the average distance fromt the desired encoder amount
                double output = pid.getPid(
                ((targetLeftEncoder - Odometery::getEncoder(LEFT_MOTOR_FRONT)) + (targetRightEncoder - Odometery::getEncoder(RIGHT_MOTOR_FRONT))) / 2);
                
                Controller.print(0, 0, "%f", output);
                
                double headingDiff = startRotation - pose.rotation;
                double headingOutput = headingDiff * headingKp;
                                
                if(fabs(output) > maxVoltage)
                {
                    output = sign(output) * maxVoltage;
                }
                
                //Outputting voltage into motors and slewing
                this->setVoltage(output + headingOutput, output - headingOutput, true, 10);

                //Updating async variables
                this->distanceTraveled += pointToPointDistance(Point(pose.x, pose.y), Point(prevPose.x, prevPose.y));

                //Exit conditions
                //Getting Current Motor Velocity
                double currentVel = (leftFrontMotor.get_actual_velocity() + rightFrontMotor.get_actual_velocity()) / 2;

                //Checking if error and velocity are within a certain threshold
                if(exitConditions::rangeExit(pid.getError(), errorExit) && exitConditions::rangeExit(currentVel, velExit))
                    break;

                pros::delay(10);
            }
            this->inMotion = false;
        }
    }
}