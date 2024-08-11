#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/odom.hpp"
#include "harryLibHeader/globals.h"
#include "harryLibHeader/exitConditions.hpp"
#include "harryLibHeader/util.hpp"

namespace subsystems
{
    //A REAL 1D CONTROLLER!!!??!
    void drivetrain::drive(double distance, bool async)
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
                0,  //Kp
                0,  //Ki
                0,  //Kd
                0,  //Windup Range
                0   //Max Integral
            );

            //Exit Conditions
            double errorExit = 0;
            double velExit = 0;

            //Static variables for derivatives
            Pose prevPose = this->pose;

            //Calculating Target Encoder amounts
            double encoderDistance = (distance / (DRIVE_WHEEL_DIAMETER * M_PI) * 360 * (1 / DRIVE_GEAR_RATIO));
            std::pair<double, double> targetEncoderValues = std::make_pair(
                Odometery::getEncoder(LEFT_MOTOR_FRONT) + encoderDistance,
                Odometery::getEncoder(RIGHT_MOTOR_FRONT) + encoderDistance
            );

            while(true)
            {   
                //Getting the voltage output based on the average distance fromt the desired encoder amount
                double output = pid.getPid(
                (Odometery::getEncoder(LEFT_MOTOR_FRONT) + Odometery::getEncoder(RIGHT_MOTOR_FRONT) - targetEncoderValues.first - targetEncoderValues.second) / 2);

                //Outputting voltage into motors and slewing
                this->setVoltage(output, output, true, 10);

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
    };
}