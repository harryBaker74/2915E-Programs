#include "../../include/main.h"
#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/exitConditions.hpp"

namespace boomerang
{
    Point getGhost(Point initialCarrot, Point currentCarrot, double lead)
    {
        Point ghost (0, 0);

        ghost.x = initialCarrot.x + (currentCarrot.x - initialCarrot.x) * (1 - lead);
        ghost.y = initialCarrot.y + (currentCarrot.y - initialCarrot.y) * (1 - lead);

        return ghost;
    }

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
    void drivetrain::boomerang(Pose targetPose, double dLead, double gLead, bool backwards, bool radians, bool async)
        {
            //Prevent this motion from starting if the robot is already in a motion
            while(inMotion)
                pros::delay(10);


            if (async)
            {
                pros::Task task {[=, this] {
                    boomerang(targetPose, dLead, gLead, backwards, radians, false);
                    pros::Task::current().remove();
                }};
            }
            else
            {   
                this->distanceTraveled = 0;
                this->inMotion = true;

                //PID's
                PID::PID angPid = PID::PID(
                    700.0,    //Kp
                    0.0,    //Ki
                    1000.0,    //Kd
                    0.0,    //Windup Range
                    0.0     //Max Intergal
                );
                PID::PID linPid = PID::PID(
                    15.0,
                    0.0,
                    150.0,
                    0.0,
                    0.0
                );

                //Switch conditions
                //The distance at which the robot needs to be from the current target point, in order to move onto the next target point
                double switchDistance = 10;
                //Radius of exit semicircle, prob should be automatic
                double exitDistance = 3.5;
                double velExit = 0;

                //Angular Falloff Parameter
                double angK = 13;

                //Angular switch parameter
                //This parameter decides when to switch from facing the carrot point to facing the heading
                //Lower values means that the switch will happen quicker, with 0 being no switch
                //Eg:   angSwitch = 1, 50, 50 weighted for hypot = 1;
                //      angSwitch = 2, 50, 50 weighted for hypot = 2;
                double angSwitch = 8;

                //Converting to radians if needed
                if(!radians) targetPose.heading *= M_PI / 180;  //Converting to radians if needed

                //Updating varibales for bakwards movement
                int linMultiplier = backwards ? -1 : 1;

                targetPose.heading = backwards ? boundAngle(targetPose.heading + M_PI, true) : targetPose.heading;

                //Caluclating initial carrot point
                Point initialCarrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                //Initialixing carrot point
                Point carrot = initialCarrot;

                //Switch boolean
                bool followGhost = true;

                //Initializing Velocity Controllers
                vController::vController leftVCon(true);
                vController::vController notLeftVCon(true);

                //Prev velocities for velocity controllers
                double prevLeftVel = 0;
                double prevRightVel = 0;
                
                while(true)
                {  
                    //Deciding what carrot point to use
                    //Inspired by https://github.com/Pixel-Lib/Pixel/blob/main/src/pxl/movements/boomerang.cpp
                    carrot = boomerang::getCarrot(this->pose, targetPose, dLead);

                    //Calculating offset from carrot point
                    Point deltaPos(carrot.x - this->pose.x, carrot.y - this->pose.y);

                    //Converts delta cartesian coordinates to polar coordinates, than takes theta and adds pi/2 to it to convert it to +y = 0, then bounds the angle to -pi/pi;
                    double targetHeading = atan3(deltaPos.y, deltaPos.x);
                    //Adding 180 deg to target heading if we're driving backwards
                    targetHeading = backwards ? boundAngle(targetHeading + M_PI, true) : targetHeading;

                    //Caluclating distance to carrot
                    double distance = sqrt(pow(deltaPos.x, 2) + pow(deltaPos.y, 2));

                    //Figures out the nearest multiple of the difference between the target heading and current heading, to the current rotation
                    double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);
                    //Setting the target pose' rotation the same way
                    targetPose.rotation = pose.rotation + boundAngle(targetPose.heading - pose.heading, true);
                    
                    //Setting the target rotation to a weighted average between the rotation to the carrot, and the end target rotation
                    //Based off of angSwitch;
                    targetRotation = getWeightedAverage(targetRotation, targetPose.rotation, fabs(distance) / (fabs(distance) + angSwitch));

                    //Calculating angMultiplier
                    //Lowers the angular velocity the closer you get to the end point, preventing spinning in circles
                    double angMultiplier = fabs(distance) / (fabs(distance) + angK);

                    //Calculate velocities
                    double angVel = angPid.getPid(this->pose.rotation, targetRotation) * angMultiplier;
                    double linVel = std::fmin(linPid.getPid(distance) * cos(std::fmin(fabs(targetRotation - this->pose.rotation), 90) * angMultiplier), 600 - angVel) * linMultiplier;
                    double leftVel = linVel + angVel;
                    double rightVel = linVel - angVel;
                    //Getting current velocities
                    double currentLeftVel = leftFrontMotor.get_actual_velocity();
                    double currentRightVel = rightFrontMotor.get_actual_velocity();

                    //Converting Velocities to voltage
                    double leftVoltage = leftVCon.rpmVelToVoltage(currentLeftVel, prevLeftVel, leftVel);
                    double rightVoltage = notLeftVCon.rpmVelToVoltage(currentRightVel, prevRightVel, rightVel);
                    //Updating prev Velocities
                    prevLeftVel = leftVel;
                    prevRightVel = rightVel;

                    //Setting the motors voltage
                    this->setVoltage(leftVoltage, rightVoltage);

                    //Exit Conditions, Semi circle exit
                    //Should add velocity exit here in the future
                    if(exitConditions::semiCircleCheck(this->pose, Point(targetPose.x, targetPose.y), targetPose.heading, exitDistance))
                        break; //Comment out for tuning maybe

                        


                    //Updating distance traveled for async functions
                    this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));
                    
                    //Delay for other tasks
                    pros::delay(10);
                }
                this->inMotion = false;
                this->setVoltage(0, 0);
            }
        }
}