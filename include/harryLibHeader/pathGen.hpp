#pragma once

#include "harryLibHeader/util.hpp"
#include <vector>
struct cubicBezier
{
    //Very, Very heavily inspired by https://github.com/RobertRen1177/Tangent-Intersection-Path-Following-Algo-For-Wheeled-Mobile-Robots/blob/main/Math/CubicBezier.hpp

    cubicBezier(Point p1, Point p2, Point p3, Point p4);

    Point getPoint(double t);

    Point getFirstDerivative(double t);

    Point getSecondDerivative(double t);

    double getFirstDistanceDerivative(double t, Point point);

    double getSecondDistanceDerivative(double t, Point point);

    

    double smallestDistance(Point point, double initialGuess, double maxIterations = 5, double tolerance = 1e-6);

    public:
    Point p1 = Point(0, 0);
    Point p2 = Point(0, 0);
    Point p3 = Point(0, 0);
    Point p4 = Point(0, 0);
};

class profile
{
    public:

    profile(double maxVel, double maxAccel, double maxDeccel);

    //Generates a linear velocity profie, for use with steering algorithms
    std::vector<std::vector<double>> generateProfile(cubicBezier curve, double pointAmount = 50, double k = 5);
    
    double maxVel;
    double maxAccel;
    double maxDeccel;
};