#pragma once
#include "util.hpp"

namespace exitConditions
{
    bool semiCircleCheck(Pose robotPose, Point targetPoint, double targetHeading, double radius);
    bool rangeExit(double input, double exitRange);
}