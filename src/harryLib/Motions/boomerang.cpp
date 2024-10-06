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
    void drivetrain::boomerang(Pose targetPose, double minSpeed, double dLead, bool backwards, bool radians, bool async)
        {
            //Prevent this motion from starting if the robot is already in a motion
            while(inMotion)
                pros::delay(10);


            if (async)
            {
                pros::Task task {[=, this] {
                    boomerang(targetPose, minSpeed, dLead, backwards, radians, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {   
                this->distanceTraveled = 0;
                this->inMotion = true;

                //PID's
                PID::PID angPid = PID::PID(
                    25000.0,    //Kp
                    1000.0,        //Ki
                    200000.0,        //Kd
                    0.025,        //Windup Range
                    0.0         //Max Intergal
                );
                PID::PID linPid = PID::PID(
                    500.0,
                    0.0,
                    10000.0,
                    0.0,
                    0.0
                );

                //Radius of exit semicircle, prob should be automatic
                double exitDistance = 2;
                double headingExit = 0.008; //In rad
                double headingVelExit = 0.002;

                //Converting to radians if needed
                if(!radians) targetPose.heading *= M_PI / 180;

                //Calculating carrot
                Point carrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                //Settling variables
                double settleDistance = 15;
                bool close = false;
                double prevDistance = infinity(); 
                bool prevSide = false;

                while(true)
                {

                    //Updating carrot point
                    carrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                    Point deltaPos (carrot.x - pose.x, carrot.y - pose.y);

                    double distance = pointToPointDistance(pose, carrot);

                    double targetHeading = atan3(deltaPos.y, deltaPos.x);
                    double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);
                    double endingRotation = pose.rotation + boundAngle(targetPose.heading - pose.heading, true);

                    if(close)
                        targetRotation = endingRotation;

                    double angOutput = angPid.getPid(pose.rotation, targetRotation);
                    double linOutput = 0;


                    double switchCurve = fabs(distance) / (fabs(distance) + settleDistance);
                    linOutput = getWeightedAverage(linPid.getPid(distance) * cos(fabs(pose.rotation - targetRotation)), 
                                                    fmax(linPid.getPid(distance), minSpeed) * cos(fabs(pose.rotation - targetRotation)), switchCurve);

                    if((fabs(distance) <= settleDistance) && (!close))
                    {
                        close = true;
                    }

                    double leftVoltage = linOutput + angOutput;
                    double rightVoltage = linOutput - angOutput;

                    double ratio = fmax(fabs(leftVoltage), fabs(rightVoltage)) / 12000;
                    if(ratio > 1)
                    {
                        leftVoltage /= ratio;
                        rightVoltage /= ratio;
                    }

                    setVoltage(leftVoltage, rightVoltage);

                    //Exit Conditions
                    if(exitConditions::rangeExit(distance, exitDistance))
                        if(exitConditions::rangeExit(fabs(angPid.getError()), headingExit) && exitConditions::rangeExit(fabs(angPid.getDervative()), headingVelExit))
                            break;
                    
                    //Robot has gone past point
                    bool side = (pose.y - targetPose.y) * cos(targetPose.heading) >= (pose.x - targetPose.x) * -sin(targetPose.heading) - exitDistance; 
                    if(close && (prevSide != side))
                        break;
                    prevSide = side;

                    Controller.print(0, 0, "%d", side);
                    
                    //Updating distance traveled for async functions
                    this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));
                    
                    //Delay for other tasks
                    pros::delay(10);
                }
                this->inMotion = false;
            }
        }
}