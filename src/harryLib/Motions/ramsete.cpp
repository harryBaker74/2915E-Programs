#include "harryLibHeader/pathGen.hpp"
#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/globals.h"

namespace subsystems
{
    void drivetrain::followPath(trajectory path, double lookahedDist)
    {
        //Figuring out number of points to compare distances with
        int lookaheadAmount = floor(lookahedDist / path.ds);

        //
        int foundIndex = 0;
        int smallestIndex = 0;
        double smallestDistance = MAXFLOAT;

        double beta = 2.0;
        double zeta = 0.7;
        
        while(true)
        {
            //Finding closest point
            smallestDistance = MAXFLOAT;
            for(int i = foundIndex; i < (foundIndex + lookaheadAmount); i++)
            {
                if (i >= path.points.size())
                    break;
                
                double distance = pointToPointDistance(pose, path.points.at(i).pose);

                if(distance < smallestDistance)
                {
                    smallestDistance = distance;
                    smallestIndex = i;
                }
            }

            foundIndex = smallestIndex;

            //Ramsete velocity calculations
                //Calculating local error
                Pose globalError((path.points.at(foundIndex).pose.x - pose.x), (path.points.at(foundIndex).pose.y - pose.y), (path.points.at(foundIndex).pose.heading - pose.heading));
                Pose localError(
                    (globalError.x * cos(pose.rotation)) + (globalError.y * -sin(pose.rotation)),
                    (globalError.x * sin(pose.rotation)) + (globalError.y * cos(pose.rotation)),
                    ((boundAngle(globalError.heading, true)))
                );

                //Calculating Velocities
                double pathLinVel = path.points.at(foundIndex).linVel;
                double pathAngVel = path.points.at(foundIndex).angVel;

                //Divide linVel by 100 to convert from cm/s to m/s so i can use wpilib constants
                double k = 2 * zeta * sqrt(pow(pathAngVel, 2) + (beta * pow(pathLinVel / 100, 2)));
                double linVel = (pathLinVel * cos(localError.heading)) + (k * localError.y);
                double angVel = (pathAngVel) + (k * localError.heading) + ((beta * pathLinVel * sin(localError.heading) * localError.x) / (localError.heading)); 

            //Converting to left right velocities
                double leftVel = pathLinVel + (pathAngVel * (TRACKWIDTH * 2.54) / 2);
                double rightVel = pathLinVel - (pathAngVel * (TRACKWIDTH * 2.54) / 2);

            //Converting from m/s to rpm for motors 
                leftVel = ((leftVel / (DRIVE_WHEEL_DIAMETER * 2.54 * M_PI)) / DRIVE_GEAR_RATIO) * 60;  
                rightVel = ((rightVel / (DRIVE_WHEEL_DIAMETER * 2.54 * M_PI)) / DRIVE_GEAR_RATIO) * 60;  
            
            //Moving motors - should use actually velocity controller here
                leftDriveMotors.move_velocity(leftVel);
                rightDriveMotors.move_velocity(rightVel);


            // printf("I:%d, Pose:(%.2f, %.2f), Point:(%.2f, %.2f) Vel:(%.2f, %.2f)\n", lastFoundIndex, testPose.x, testPose.y, path.points.at(lastFoundIndex).position.x, path.points.at(lastFoundIndex).position.y, leftVel, rightVel);
            // testPose.set(Pose(testPose.x, testPose.y + 21.9456, 0));
            Controller.print(0, 0, "%.2f, %.2f", pathLinVel, pathAngVel);


            // pros::delay(10);
            pros::delay(10);

            if(foundIndex >= path.points.size() - 3)
                break;
        }
        setVoltage(0, 0);
    }
}