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
    void drivetrain::boomerang(Pose targetPose, double dLead, bool backwards, bool radians, bool async)
        {
            //Prevent this motion from starting if the robot is already in a motion
            while(inMotion)
                pros::delay(10);


            if (async)
            {
                pros::Task task {[=, this] {
                    boomerang(targetPose, dLead, backwards, radians, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {   
                this->distanceTraveled = 0;
                this->inMotion = true;

                //PID's
                PID::PID angPid = PID::PID(
                    20000.0,    //Kp
                    0.0,    //Ki
                    250000.0,    //Kd
                    0.0,    //Windup Range
                    0.0     //Max Intergal
                );
                PID::PID linPid = PID::PID(
                    300.0,
                    0.0,
                    1000.0,
                    0.0,
                    0.0
                );

                //Radius of exit semicircle, prob should be automatic
                double exitDistance = 6.5;
                double headingExit = 0.2; //In rad
                double velExit = 0;

                //Angular switch parameter
                //This parameter decides when to switch from facing the carrot point to facing the heading
                //Lower values means that the switch will happen faster and later on, with 0 being no switch
                //Eg:   angSwitch = 1, 50 50 weighted for hypot = 1;
                //      angSwitch = 2, 50 50 weighted for hypot = 2;
                double angSwitch = 25;

                //Converting to radians if needed
                if(!radians) targetPose.heading *= M_PI / 180;

                //Updating varibales for backwards movement
                int linMultiplier = backwards ? -1 : 1;

                //Initializing Velocity Controllers
                vController::vController leftVCon(true);
                vController::vController notLeftVCon(true);

                //Prev velocities for velocity controllers
                double prevLeftVel = 0;
                double prevRightVel = 0;
                
                while(true)
                {
                    //Calculating carrot point
                    Point carrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                    //Calculating targte point, interpolating between carrot and end, prevernts slow downs with high d lead
                    double distancetoEnd = sqrt(pow(targetPose.x - pose.x, 2) + pow(targetPose.y - pose.y, 2));
                    Point targetPoint = Point(getWeightedAverage(carrot.x, targetPose.x, fabs(distancetoEnd) / (fabs(distancetoEnd) + angSwitch)), 
                                         getWeightedAverage(carrot.y, targetPose.y, fabs(distancetoEnd) / (fabs(distancetoEnd) + angSwitch)));

                    //Calculating offset from carrot point
                    Point deltaPos(targetPoint.x - this->pose.x, targetPoint.y - this->pose.y);

                    //Converts delta cartesian coordinates to polar coordinates, than takes theta and adds pi/2 to it to convert it to +y = 0, then bounds the angle to -pi/pi;
                    double targetHeading = atan3(deltaPos.y, deltaPos.x);
                    //Adding 180 deg to target heading if we're driving backwards
                    targetHeading = backwards ? boundAngle(targetHeading + M_PI, true) : targetHeading;

                    //Caluclating distance to carrot
                    double distance = sqrt(pow(deltaPos.x, 2) + pow(deltaPos.y, 2));

                    //Figures out the nearest multiple of the difference between the target heading and current heading, to the current rotation
                    double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);
                    //Setting the target pose' rotation the same way
                    targetPose.rotation = pose.rotation + boundAngle(boundAngle(targetPose.heading + (M_PI * backwards), true) - pose.heading, true);
                    
                    //Setting the target rotation to a weighted average between the rotation to the carrot, and the end target rotation
                    //Based off of angSwitch
                    //Gets rid of spinning in circles on point aswell, so dont need angK
                    targetRotation = getWeightedAverage(targetRotation, targetPose.rotation, fabs(distance) / (fabs(distance) + angSwitch));

                    //Calculate velocities
                    double angOutput = angPid.getPid(this->pose.rotation, targetRotation);
                    double linOutput = std::fmax(std::fmin(linPid.getPid(distance), 12000 * cos(std::fmin(fabs(targetRotation - pose.rotation), 90)) - angOutput), 0) * linMultiplier;

                    Controller.print(0, 0, "%.3f", angPid.getError());

                    double leftVoltage = linOutput + angOutput;
                    double rightVoltage = linOutput - angOutput;

                    //Setting the motors voltage
                    this->setVoltage(leftVoltage, rightVoltage);

                    //Exit Conditions, Semi circle exit
                    //Should add velocity exit here in the future
                    if(exitConditions::semiCircleCheck(this->pose, Point(targetPose.x, targetPose.y), targetPose.heading, exitDistance) && exitConditions::rangeExit(angPid.getError(), headingExit))
                        break; //Comment out for tuning maybe


                    //Updating distance traveled for async functions
                    this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));
                    
                    //Delay for other tasks
                    pros::delay(10);
                }
                this->inMotion = false;
            }
        }
}