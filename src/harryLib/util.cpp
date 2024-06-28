#include "../include/main.h"
#include "../include/harryLibHeader/util.hpp"
#include <vector>

int sign(double num)
{
    if (num >= 0)
        return 1;
    else
        return -1;
}

double getMinAngle(double targetAngle, double currentAngle, bool radians)
{
    double angleDifference;

    if (radians)//Radians
    {
        angleDifference = targetAngle - currentAngle;
        if ((angleDifference > M_PI) || (angleDifference < -1 * M_PI))
            angleDifference = -1 * sign(angleDifference) * ((2 * M_PI) - fabs(angleDifference));
    }
    else//Degrees
    {
        double angleDifference = targetAngle - currentAngle;
        if ((angleDifference > 180) || (angleDifference < -1 * 180))
            angleDifference = -1 * sign(angleDifference) * (360 - fabs(angleDifference));
    }
    
    return angleDifference;
}

double slew(double targetAmount, double currentAmount, double rateOfChange_ms, double timeStep_ms)
{
    double desiredChange = targetAmount - currentAmount;
    double maxChange = rateOfChange_ms * timeStep_ms;

    return fabs(desiredChange) > fabs(maxChange) ? maxChange : desiredChange;
}

double pointToPointDistance(Point p1, Point p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

double linearToCubed(double input, double maxInput, double k)
{
    return k * (pow(input, 3) / pow(maxInput, 2));
}

//Point struct
Point::Point(double x, double y)
{
    this->x = x;
    this->y = y;
}

void Point::set(double x, double y)
{
    this->x = x;
    this->y = y;
}

//Pose Struct
Pose::Pose(double x, double y, double heading)
{
    this->x = x;
    this->y = y;
    this->heading = heading;
}
void Pose::set(double x, double y, double heading)
{
    this->x = x;
    this->y = y;
    this->heading = heading;
}
void Pose::set(Pose pose)
{
    this->x = pose.x;
    this->y = pose.y;
}
