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
    void drivetrain::drive(double distance, double heading, double timeout_ms, double maxVoltage, double errorExit, bool async)
    {
        //Prevent multiple motions from running at once
        while (inMotion)
            pros::delay(10);

        if (async)
        {
            pros::Task task {[=, this]{
                drive(distance, heading, timeout_ms, maxVoltage, errorExit, false);
                pros::Task::current().remove();
            }};
        }
        else
        {
            this->distanceTraveled = 0;
            this->inMotion = true;

            //Tuning
            PID::PID linPid = PID::PID(
                10,  //Kp
                0.3,  //Ki
                80,  //Kd
                2,  //Windup Range
                0   //Max Integral
            );

            PID::PID angPid = PID::PID{
                500,  //Kp
                0,  //Ki
                3000,  //Kd
                0,  //Windup Range
                0   //Max Integral
            };

            std::vector<double> leftEncoders = Odometery::getLeftDriveEncoders();
            std::vector<double> rightEncoders = Odometery::getRightDriveEncoders();

            double leftOffset = (leftEncoders.at(0) + leftEncoders.at(1) + leftEncoders.at(2)) / 3;
            double rightOffset = (rightEncoders.at(0) + rightEncoders.at(1) + rightEncoders.at(2)) / 3;

            // double settleDistance = 1; 
            // bool prevSide = false;

            double endTime = pros::millis() + timeout_ms;            
            while(endTime >= pros::millis())
            {   

                leftEncoders = Odometery::getLeftDriveEncoders();
                rightEncoders = Odometery::getRightDriveEncoders();

                //Getting the voltage output based on the average distance fromt the desired encoder amount
                // double linOutput = linPid.getPid(pointToPointDistance(pose, point));

                double currentLeftEncoder = ((leftEncoders.at(0) + leftEncoders.at(1) + leftEncoders.at(2)) / 3) - leftOffset;
                double currentRightEncoder = ((rightEncoders.at(0) + rightEncoders.at(1) + rightEncoders.at(2)) / 3) - rightOffset;

                double avgDist = ((currentLeftEncoder + currentRightEncoder) / 2) * M_PI * DRIVE_WHEEL_DIAMETER * 2.54 * DRIVE_GEAR_RATIO / 360;

                double linOutput = linPid.getPid(distance - avgDist);

                linOutput = std::clamp(linOutput, -maxVoltage, maxVoltage);

                double targetRotation = pose.rotation + boundAngle(heading + ((distance < 0) * M_PI) - pose.heading, true);
                
                double angOutput = angPid.getPid(boundAngle(heading + ((distance < 0) * M_PI) - pose.heading, true));

                double leftVoltage = (linOutput + angOutput) / 127 * 12000;
                double rightVoltage = (linOutput - angOutput) / 127 * 12000;

                double ratio = fmax(fabs(leftVoltage), fabs(rightVoltage)) / (maxVoltage / 127 * 12000);

                if(ratio > 1)
                {
                    leftVoltage /= ratio;
                    rightVoltage /= ratio;
                }

                //Outputting voltage into motors and slewing
                setVoltage(leftVoltage, rightVoltage);

                //Updating async variables
                this->distanceTraveled = avgDist;

                Controller.print(0, 0, "%f", leftFrontMotor.get_position());

                //Exit conditions
                //Checking if error and velocity are within a certain threshold
                if(fabs(linPid.getError()) <= errorExit)
                {
                    break;
                }

                pros::delay(10);
            }
            setVoltage(0, 0);
            this->inMotion = false;
        }
    }

    void drivetrain::drive(double distance, double heading, double maxVoltage)
    {
        drive(distance, heading * M_PI / 180, 10000, maxVoltage, 1, false);
    }

    void drivetrain::driveToPoint(Point point, bool backwards, double timeout_ms, double maxVoltage, bool async)
    {
        // double distance = pointToPointDistance(point, pose);
        // drive(distance, timeout_ms, maxVoltage, async);

        while (inMotion)
            pros::delay(10);
        

        if (async)
        {
            pros::Task task {[=, this]{
                driveToPoint(point, timeout_ms, maxVoltage, false);
                pros::Task::current().remove();
            }};
        }
        else
        {
            this->distanceTraveled = 0;
            this->inMotion = true;

            //Tuning
            PID::PID linPid = PID::PID(
                1000,  //Kp
                0,  //Ki
                8000,  //Kd
                0,  //Windup Range
                0   //Max Integral
            );

            PID::PID angPid = PID::PID(
                19000,
                0,
                80000,
                0,
                0
            );

            double settleDistance = 5; 
            bool close = false;

            //                 cm
            //Exit Conditions   ^
            double errorExit = 3;
            double velExit = 0.25;

            double headingErrorExit = 1;

            int counter = 0;
            double endTime = pros::millis() + timeout_ms;
            
            while(/*endTime >= pros::millis()*/true)
            {   
                //Getting the voltage output based on the average distance fromt the desired encoder amount
                double linOutput = linPid.getPid(pointToPointDistance(pose, point)) * (backwards == 0 ? 1 : -1);

                //Calculating angular and linear error#
                double targetHeading = M_PI_2 - atan2(point.y - pose.y, point.x - pose.x);
                double targetRotation = pose.rotation + boundAngle(targetHeading + (backwards * M_PI) - pose.heading, true);

                if(pointToPointDistance(pose, point) < settleDistance)
                    close = true;

                // //If more than 90 deg error
                // if((fabs(targetRotation) > M_PI_2) && close)
                // {
                //     //Flip heading to other side, and reverse to point
                //     targetRotation = pose.rotation + boundAngle(targetHeading + M_PI, true);
                //     linOutput *= -1;
                // }

                double angOutput =  angPid.getPid(pose.rotation, targetRotation);

                angOutput *= fmin(pointToPointDistance(pose, point) / settleDistance, 1);
                linOutput *= fabs(cos(fmin(fabs(4 * angPid.getError()), M_PI_2)));

                double leftVoltage = linOutput + angOutput;
                double rightVoltage = linOutput - angOutput;

                double ratio = fmax(fabs(leftVoltage), fabs(rightVoltage)) / maxVoltage;
                if(ratio > 1)
                {
                    leftVoltage /= ratio;
                    rightVoltage /= ratio;
                }

                //Outputting voltage into motors and slewing
                this->setVoltage(leftVoltage, rightVoltage);

                //Updating async variables
                this->distanceTraveled += pointToPointDistance(prevPose, pose);

                //Exit conditions
                //Getting Current Motor Velocity
                double currentVel = (leftFrontMotor.get_actual_velocity() + rightFrontMotor.get_actual_velocity()) / 2;

                //Checking if error and velocity are within a certain threshold
                if(exitConditions::rangeExit(linPid.getError(), errorExit))
                {
                    break;
                }

                pros::delay(10);
            }
            setVoltage(0, 0);
            this->inMotion = false;
        }
    }
}