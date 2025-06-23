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
    else if ((y == 0) && (x < 0))
    {
        return -M_PI_2;
    }
    else if ((y == 0) && (x > 0))
    {
        return M_PI_2;
    }
    else if ((x == 0) && (y < 0))
    {
        return -M_PI;
    }


    //Undefined, or (x == 0) && (y > 0)
    return 0;
}

double boundAngle(double angle, bool radians)
{
    if (radians)
    {
        while ((angle > M_PI) || (angle < -1 * M_PI))
            angle = -1 * sign(angle) * ((2 * M_PI) - fabs(angle));
    }
    if(!radians)
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

//Returns curvature between 2 poses, from lemlib
float getCurvature(Pose pose, Pose other) 
{
    double thetaCCW = boundAngle(M_PI_2 - pose.heading, true);

    // calculate whether the pose is on the left or right side of the circle
    float side = sign(sin(pose.heading) * (other.x - pose.x) - cos(pose.heading) * (other.y - pose.y));

    // calculate center point and radius
    float a = -std::tan(pose.heading);
    float c = std::tan(pose.heading) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

//Point struct
Point::Point()
{
    this->x = 0;
    this->y = 0;
}


Point::Point(double x, double y)
{
    this->x = x;
    this->y = y;
}

Point::Point(Pose pose)
{
    this->x = pose.x;
    this->y = pose.y;
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
Pose::Pose(double x, double y, double heading , double rotation)
{
    this->x = x;
    this->y = y;
    this->heading = heading;
    this->rotation = rotation;
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
