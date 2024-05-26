#include "../include/main.h"
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

double pointToPointDistance(std::vector<double> p1, std::vector<double> p2)
{
    return sqrt(pow(p2.at(0) - p1.at(0), 2) + pow(p2.at(1) - p1.at(1), 2));
}