#pragma once
#include "main.h"
namespace Odometery
{
    void OdometeryCalculations(Pose* robotPose, pros::MotorGroup* leftDrive, pros::MotorGroup* rightDrive, pros::Rotation* trackingWheel, pros::IMU* IMU);
}