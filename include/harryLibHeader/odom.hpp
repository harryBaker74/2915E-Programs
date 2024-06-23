#pragma once
#include "main.h"
namespace Odometery
{
    Pose OdometeryCalculations(Pose robotPose, pros::MotorGroup* leftDrive, pros::MotorGroup* rightDrive);
}