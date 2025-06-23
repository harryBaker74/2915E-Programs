#include "../include/main.h"
#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/exitConditions.hpp"

namespace boomerang
{
    Point getCarrot(Pose robotPose, Pose targetPose, double lead)
    {
        Pose deltaPose (0, 0, 0);
        Point carrot (0, 0);

        //Calculating delta cartesian offesets
        deltaPose.x = targetPose.x - robotPose.x;
        deltaPose.y = targetPose.y - robotPose.y;

        //Caluclating distance to end point
        double hypot = sqrt(pow(deltaPose.x, 2) + pow(deltaPose.y, 2));

        //Calculating carrot point position
        carrot.x = targetPose.x - hypot * sin(targetPose.heading) * lead;
        carrot.y = targetPose.y - hypot * cos(targetPose.heading) * lead;

        return carrot;
    }
}

namespace subsystems
{
    void drivetrain::boomerang(Pose targetPose, double maxSpeed, double dLead, bool backwards, bool radians, bool async, int timeout_ms)
        {
            //Prevent this motion from starting if the robot is already in a motion
            while(inMotion)
                pros::delay(10);


            if (async)
            {
                pros::Task task {[=, this] {
                    boomerang(targetPose, maxSpeed, dLead, backwards, radians, false, timeout_ms);
                    pros::Task::current().remove();
                }};
            }
            else
            {   
                this->distanceTraveled = 0;
                this->inMotion = true;

                //PID's
                PID::PID angPid = PID::PID(
                    // 20000.0,    //Kp
                    // 500.0,        //Ki
                    // 200000.0,        //Kd
                    // 0.025,        //Windup Range
                    // 0.0         //Max Intergal
                    13000,
                    0,
                    75000,
                    0,
                    0
                );

                PID::PID linPid = PID::PID(
                    1500,//400.0,
                    0.0,
                    7500,//5000.0,
                    0.0,
                    0.0
                );

                //Converting to radians if needed
                if(!radians) 
                {
                    targetPose.heading *= M_PI / 180;
                    targetPose.rotation *= M_PI / 180;
                }

                //Calculating carrot
                Point carrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                double linExit = 4;
                double angExit = 0.01;
                double earlyExitRange = 1.5;
                double horizontalDrift = 8;

                int linMultiplier = backwards ? -1 : 1;

                bool close = false;
                double prevLinOut = 0;
                bool prevSameSide = false;

                int endTime = pros::millis() + timeout_ms;

                while(pros::millis() < endTime)
                {
                    //Getting distance to target point
                    float distanceToTarget = pointToPointDistance(pose, targetPose);

                    //If close to target point, activate settling behaviour
                    if(distanceToTarget <= (7.5 * 2.54) && !close)
                    {
                        close = true;
                        // maxSpeed = fmax(fabs(prevLinOut), 5669);
                    }

                    //Calculating carrot
                    carrot = boomerang::getCarrot(pose, targetPose, dLead);
                    Point actualCarrot = carrot; 
                    //For settling
                    if(close)
                        carrot = Point(targetPose);


                    //Calculating angular and linear error#
                    double targetHeading = M_PI_2 - atan2(carrot.y - pose.y, carrot.x - pose.x);
                    double targetRotation = pose.rotation + boundAngle(targetHeading + (backwards * M_PI) - pose.heading, true);
                    double endingRotation = pose.rotation + boundAngle(targetPose.heading + (backwards * M_PI) - pose.heading, true);

                    double angError = targetRotation - pose.rotation;
                    double linError = pointToPointDistance(carrot, pose) * linMultiplier;

                    //Updating angError if we're close
                    if (close)
                        angError = endingRotation - pose.rotation;

                    //From lemlib
                    //Use cos for settling, sign(cos) otherwise
                    //maxSliSpeed helps when not settling
                    // if(close)
                    //     linError *= cos(angError);
                    // else
                    //     linError *= sign(cos(angError));


                    


                    //Getting outputs form pids
                    double linOutput = linPid.getPid(linError);
                    double angOutput = angPid.getPid(angError);


                    //Calamping to max speed
                    angOutput = std::clamp(angOutput, -maxSpeed, maxSpeed);
                    linOutput = std::clamp(linOutput, -maxSpeed, maxSpeed);

                    prevLinOut = linOutput;

                    //Multiplying maxSpeed by either cos of 2 * error to carrot, or cos of error to final heading, min of 0.1 multiply to prevent stopping or reversing 
                    double maxLinSpeed = maxSpeed * fmin(fmax(cos(2 * (targetRotation - pose.rotation)), 0.1), fmax(cos(fabs(endingRotation- pose.rotation)), 0.1));
                    linOutput = std::clamp(linOutput, -maxLinSpeed, maxLinSpeed);

                    printf("LinO:%.2f, maxSpeed:%.2f, maxLin:%.2f\n", linOutput, maxSpeed, maxLinSpeed);


                    //Prioritizing ang over lin
                    double overturn = fabs(angOutput) + fabs(linOutput) - maxSpeed;
                    if(overturn > 0)
                        linOutput -= sign(linOutput);

                    //Prevent reversing
                    if(!backwards && !close)
                        linOutput = fmax(linOutput, 0);
                    else if(backwards && !close)
                        linOutput = fmin(linOutput, 0);

                    //setting left and right and ratioing them correctly



                    double leftOutput = linOutput + angOutput;
                    double rightOutput = linOutput - angOutput;
                    double ratio = fmax(fabs(leftOutput), fabs(rightOutput)) / maxSpeed;
                    if(ratio > 1)
                    {
                        leftOutput /= ratio;
                        rightOutput /= ratio;
                    }

                    //Moving motors
                    setVoltage(leftOutput, rightOutput);


                    //lin exit conditions
                    //Angular exit conditions
                    if(fabs(linPid.getError()) < linExit && fabs(angPid.getError()) < angExit && close)
                        break;
                    //Robot, carrot side exit conditions
                    double adjustedAngle = boundAngle(M_PI_2 - targetPose.heading, true);
                    bool robotSide = (pose.y - targetPose.y) * -sin(adjustedAngle) <= (pose.x - targetPose.x) * cos(adjustedAngle) + earlyExitRange;
                    bool carrotSide =  (actualCarrot.y - targetPose.y) * -sin(adjustedAngle) <= (actualCarrot.x - targetPose.x) * cos(adjustedAngle) + earlyExitRange;
                    bool sameSide = robotSide == carrotSide;

                    if (!sameSide && prevSameSide && close) 
                         break;

                    prevSameSide = sameSide;


                    this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));

                    pros::delay(20);
                }
                this->inMotion = false;
                printf("\n\n\n\n\n\n--------------------------------------------------------------------------------------------------------\n\n\n\n\n\n");


                // double horizontalDrift = 12;

                // //Radius of exit semicircle, prob should be automatic
                // double exitDistance = 1;
                // double headingExit = 0.008; //In rad
                // double headingVelExit = 0.002;

                // //Converting to radians if needed
                // if(!radians) 
                // {
                //     targetPose.heading *= M_PI / 180;
                //     targetPose.rotation *= M_PI / 180;
                // }
                // //Calculating carrot
                // Point carrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                // //For backwards movement
                // int linMultiplier = backwards ? -1 : 1;

                // //Settling variables
                // double settleDistance = 15;
                // bool close = false;
                // double prevDistance = infinity(); 
                // bool prevSide = false;

                // int endTime = pros::millis() + timeout_ms;

                // while(pros::millis() < endTime)
                // {
                //     //Updating carrot point
                //     carrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                //     Point deltaPos (carrot.x - pose.x, carrot.y - pose.y);

                //     double distance = pointToPointDistance(pose, carrot);

                //     double targetHeading = atan3(deltaPos.y, deltaPos.x);
                //     double targetRotation = pose.rotation + boundAngle(targetHeading + (backwards * M_PI) - pose.heading, true);
                //     double endingRotation = pose.rotation + boundAngle(targetPose.heading + (backwards * M_PI) - pose.heading, true);

                //     if(close)
                //         targetRotation = endingRotation;
                    

                //     double angOutput = angPid.getPid(pose.rotation, targetRotation);

                //     double switchCurve = fabs(distance) / (fabs(distance) + settleDistance);

                //     //Choose get biggest angle error between either robot heading and angle to carrot, and robot heading and ending heading
                //     //When the robot is either high angle error to carrot, or high angle error to end heading, drive slower
                //     double cosInput = fmax(fabs(pose.rotation - targetRotation), fabs(pose.rotation - endingRotation));

                //     double linOutput = linPid.getPid(distance) * fmax(cos(fabs(cosInput)), 0) * linMultiplier;

                //     if((fabs(distance) <= settleDistance) && (!close))
                //     {
                //         close = true;
                //     }

                //     // Limiting speed based on curvature of path, from lemlib
                //         float radius = 1 / fabs(getCurvature(pose, Pose(carrot.x, carrot.y, 0)));
                //         float maxSlipSpeed = sqrt(horizontalDrift * radius * 9.8) * 12000 / 127;
                //         if(fabs(linOutput) > maxSlipSpeed)
                //             linOutput = sign(linOutput) * maxSlipSpeed;

                //     Controller.print(0, 0, "%.2f, %.2f", deltaPos.x, deltaPos.y);

                //     double leftVoltage = linOutput + angOutput;
                //     double rightVoltage = linOutput - angOutput;

                //     double ratio = fmax(fabs(leftVoltage), fabs(rightVoltage)) / 12000;
                //     if(ratio > 1)
                //     {
                //         leftVoltage /= ratio;
                //         rightVoltage /= ratio;
                //     }

                //     setVoltage(leftVoltage, rightVoltage);

                //     //Exit Conditions
                //         if(exitConditions::rangeExit(distance, exitDistance))
                //             if(exitConditions::rangeExit(fabs(angPid.getError()), headingExit) && exitConditions::rangeExit(fabs(angPid.getDervative()), headingVelExit))
                //                 break;

                //         //Robot has gone past point
                //         bool side = (pose.y - targetPose.y) * cos(targetPose.heading) >= (pose.x - targetPose.x) * -sin(targetPose.heading) - exitDistance; 
                //         if(close && (prevSide != side))
                //             break;
                //         prevSide = side;
                    
                //     //Updating distance traveled for async functions
                //     this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));
                    
                //     //Delay for other tasks
                //     pros::delay(10);
                // }
            }
        }
}