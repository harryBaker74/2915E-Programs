#include "../../include/main.h"

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
}