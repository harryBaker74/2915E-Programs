#include "harryLibHeader/exitConditions.hpp"
#include "main.h"

namespace exitConditions
{
    bool semiCircleCheck(Pose robotPose, Point targetPoint, double targetHeading, double radius)
    {  

        Point deltaPos (robotPose.x - targetPoint.x, robotPose.y - targetPoint.y);
        double distance = sqrt(pow(deltaPos.x, 2) + pow(deltaPos.y, 2));
        //Check if robot is in the full circle, if not, then cant be in the semi circle
        if(distance > radius)
            return false;
        
        //Caluclate angle from target to robot
        double angle = atan3(deltaPos.y, deltaPos.x);
        printf("Angle:%f", angle * 180 / M_PI);
        printf("Shiatss:%f", (targetHeading - angle) * 180 / M_PI);

        return fabs(angle - targetHeading) <= M_PI;
    }

    bool rangeExit(double input, double exitRange)
    {
        if(fabs(input) < exitRange)
            return true;
        return false;
    }
}