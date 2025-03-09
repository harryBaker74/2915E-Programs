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

struct quinticBezier
{
    Point p0;
    Point p1;
    Point p2;
    Point p3;
    Point p4;
    Point p5;

    quinticBezier();
    quinticBezier(Point p0, Point p1, Point p2, Point p3, Point p4, Point p5);
    
    Point getPoint(double t);

    Point getFirstDerivative(double t);

    Point getSecondDerivative(double t);

    double getCurvature(double t);
};

struct quinticSpline
{
    std::vector<quinticBezier> curves = {quinticBezier()};

    quinticSpline(std::vector<std::vector<Point>> points);

    Point getPoint(double u);

    Point getFirstDerivative(double u);

    Point getSecondDerivative(double u);

    double getCurvature(double u);
};

struct waypoint
{
    public:
    Pose pose = Pose(0, 0, 0, 0);
    double linVel;
    double angVel;
    double u;

    //Constructor
    waypoint();
    waypoint(Pose pose, double linVel, double angVel, double u);

};

struct trajectory
{
    std::vector<waypoint> points = {};

    double ds;

    //Constructor
    trajectory(std::vector<waypoint> points, double ds);
    trajectory(std::vector<std::pair<Pose, std::vector<double>>> points, double ds);
};

class profile
{
    public:

    profile(double maxVel, double maxAccel, double maxDeccel);

    //Generates a linear velocity profie, for use with steering algorithms
    std::vector<std::vector<double>> generateProfile(cubicBezier curve, double pointAmount = 50, double k = 5);

    /**
     * @brief Generates a 2d motion profiled trajectory, based off of an inputedd spline
     * 
     * @param spline Spline to use, quintic bezier spline only
     * @param ds Delta distance between each point in cm, defaulted to 3 cm
     * @param k Tuning value for linear speed at higher curvatures, doesnt change angular velocites
     * 
     * @return A vector of pairs, with each pair having a pose, and a vector of linear and angular velocities
     */
    trajectory generateProfile(quinticSpline spline, double ds = 3, double k = 5);
    
    double maxVel;
    double maxAccel;
    double maxDeccel;
};