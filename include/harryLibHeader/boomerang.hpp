#pragma once
#include "util.hpp"
namespace boomerang
{
    Point getGhost(Point initialCarrot, Point currentCarrot, double lead);
    Point getCarrot(Pose robotPose, Pose targetPose, double lead);
    bool semiCircleCheck(Pose robotPose, Point targetPoint, double targetHeading, double radius);
}