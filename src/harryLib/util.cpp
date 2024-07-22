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

double atan3(double y, double x)
{
    if (y > 0)//Top 2 qudrants
    {
        return atan(x / y);
    }
    else if(x > 0 && y < 0)//Lower right quadrant
    {
        return atan(x / y) + M_PI;
    }
    else if(x < 0 && y < 0)//Lower LEFT!!!!!! quadrant
    {
        return atan(x / y) - M_PI;
    }
    
    //Undefined
    return 0;
}

double boundAngle(double angle, bool raidians)
{
    if (raidians)
    {
        while ((angle > M_PI) || (angle < -1 * M_PI))
            angle = -1 * sign(angle) * ((2 * M_PI) - fabs(angle));
    }
    if(!raidians)
    {
        while((angle > 180) || (angle < -180))
            angle = -1 * sign(angle) * (360 - fabs(angle));
    }

    return angle;
}

double getMinAngle(double targetAngle, double currentAngle, bool radians)
{
    double angleDifference;

    if (radians)//Radians
    {
        angleDifference = targetAngle - currentAngle;
        boundAngle(angleDifference, radians);
    }
    else//Degrees
    {
        double angleDifference = targetAngle - currentAngle;
        boundAngle(angleDifference, radians);
    }
    
    return angleDifference;
}

double inToCm(double inch)
{
    return inch * 2.54;
}

double cmToIn(double centimeter)
{
    return centimeter / 2.54;
}

double slew(double targetAmount, double currentAmount, double rateOfChange_ms, double timeStep_ms)
{
    double desiredChange = targetAmount - currentAmount;
    double maxChange = rateOfChange_ms * timeStep_ms;

    return fabs(desiredChange) > fabs(maxChange) ? (sign(desiredChange) * maxChange) + currentAmount: targetAmount;
}

double pointToPointDistance(Point p1, Point p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

double linearToCubed(double input, double maxInput, double k)
{
    return k * (pow(input, 3) / pow(maxInput, 2));
}

double lineartoSquared(double input, double maxInput, double k)
{
    return k * (pow(input, 2) * sign(input) / maxInput);
}
double getWeightedAverage(double numA, double numB, double decimalWeight)
{
    return (numA * decimalWeight) + (numB * (1 - decimalWeight));
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
