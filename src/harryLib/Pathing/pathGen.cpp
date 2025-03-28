#include "harryLibHeader/pathGen.hpp"
#include "harryLibHeader/util.hpp"
#include "harryLibHeader/globals.h"
#include "main.h"
#include <vector>

//Cubic bezier struct
//Very, Very heavily inspired by https://github.com/RobertRen1177/Tangent-Intersection-Path-Following-Algo-For-Wheeled-Mobile-Robots/blob/main/Math/CubicBezier.hpp
    cubicBezier::cubicBezier(Point p1, Point p2, Point p3, Point p4)
    {
        this->p1 = p1;
        this->p2 = p2;
        this->p3 = p3;
        this->p4 = p4;
    }

    Point cubicBezier::getPoint(double t)
    {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;
        double ttt = tt * t;
        double uuu = uu * u;

        Point point = p1 * uuu;
        point = point + p2 * (3 * uu * t);
        point = point + p3 * (3 * u * tt);
        point = point + p4 * ttt;

        return point;
    }

    Point cubicBezier::getFirstDerivative(double t)
    {
        double u = 1 - t;
        Point point = p2 - p1;
        point = point * (3 * u * u);
        point = point + (p3 - p2) * (6 * u * t);
        point = point + (p4 - p3) * (3 * t * t);

        return point;
    }

    Point cubicBezier::getSecondDerivative(double t)
    {
        return (p1 - p2 * 2 + p3) * (6 * (1 - t)) + (p2 - p3 * 2 + p4) * (6 * t);
    }

    double cubicBezier::getFirstDistanceDerivative(double t, Point point)
    {
        Point Bt = getPoint(t);
        Point BtPrime = getFirstDerivative(t);
        return ((Bt - point) * BtPrime) * 2;
    }

    double cubicBezier::getSecondDistanceDerivative(double t, Point point)
    {
        Point Bt = getPoint(t);
        Point BtPrime = getFirstDerivative(t);
        Point BtDoublePrime = getSecondDerivative(t);
        return 2 * ((BtPrime * BtPrime) + (Bt - point) * BtDoublePrime);
    }

    double cubicBezier::smallestDistance(Point point, double initialGuess, double maxIterations, double tolerance)
    {
        double t = initialGuess;
        for (int i = 0; i < maxIterations; i++)
        {
            double f = getFirstDistanceDerivative(t, point);
            double fPrime = getSecondDistanceDerivative(t, point);
            if(fabs(fPrime) < 1e-6)
                break;
            double newT = t - (f / fPrime);
            newT = fmax(0, fmin(newT, 1));
            if(fabs(t - newT) < tolerance)
                return newT;
            
            t = newT;
        }

        return t;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Quintic Bezier struct
//Inspired by cubic bezier framework, with math from https://www.desmos.com/calculator/iy5gzoasrf

    quinticBezier::quinticBezier()
    {
        this->p0 = Point(0, 0);
        this->p1 = Point(0, 0);
        this->p2 = Point(0, 0);
        this->p3 = Point(0, 0);
        this->p4 = Point(0, 0);
        this->p5 = Point(0, 0);
    }

    quinticBezier::quinticBezier(Point p0, Point p1, Point p2, Point p3, Point p4, Point p5)
    {
        this->p0 = p0;
        this->p1 = p1;
        this->p2 = p2;
        this->p3 = p3;
        this->p4 = p4;
        this->p5 = p5;
    }

    Point quinticBezier::getPoint(double t)
    {
        Point point =   
            p0 * (pow(1 - t, 5))
        +   p1 * (5 * t * pow(1 - t, 4))
        +   p2 * (10 * pow(t, 2) * pow(1 - t, 3))
        +   p3 * (10 * pow(t, 3) * pow(1 - t, 2))
        +   p4 * (5 * pow(t, 4) * (1 - t))
        +   p5 * (pow(t, 5));

        return point;
    }

    Point quinticBezier::getFirstDerivative(double t)
    {
        Point firstDerivative = 
            p0 * (-5 * pow(1 - t, 4))
        +   p1 * ((5 - (25 * t)) * pow(1 - t, 3))
        +   p2 * (t * (20 - (50 * t)) * pow(1 - t, 2))
        +   p3 * (pow(t, 2) * (30 - (50 * t)) * (1 - t))
        +   p4 * (pow(t, 3) * (20 - (25 * t)))
        +   p5 * (5 * pow(t, 4));

        return firstDerivative;
    }

    Point quinticBezier::getSecondDerivative(double t)
    {
        Point secondDerivative =
            p0 * (20 * pow(1 - t, 3))
        +   p1 * ((-40 * pow(1 - t, 3)) + ((60 * t) * pow(1 - t, 2)))
        +   p2 * ((20 * pow(1 - t, 3)) + ((-120 * t) * pow(1 - t, 2)) + ((60 * pow(t, 2)) * (1 - t)))
        +   p3 * (((60 * t) * pow(1 - t, 2)) + ((-120 * pow(t, 2)) * (1 - t)) + (20 * pow(t, 3)))
        +   p4 * (((60 * pow(t, 2)) * (1 - t)) + (-40 * pow(t, 3)))
        +   p5 * (20 * pow(t, 3));

        return secondDerivative;
    }

    double quinticBezier::getCurvature(double t)
    {
        Point firstDerivative = getFirstDerivative(t);
        Point secondDerivative = getSecondDerivative(t);

        double firstMagnitude = sqrt(pow(firstDerivative.x, 2) + pow(firstDerivative.y, 2));

        double curvature = (firstDerivative.cross(secondDerivative)) / pow(firstMagnitude, 3);

        return curvature;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Quintic Spline Struct
//Based off of the same desmos thing from the quintic bezier struct, just with some logic to handle easy continuos spline generation and interpolation

    /**
     * @brief This should be used with a spline planner of some sort, 
     * desmos link if needed: https://www.desmos.com/calculator/iy5gzoasrf
     * 
     * @param points A vector of point groups, with each point group having 3 points in it. First point in each group is the point the curve passes through, second point is first handle, third point is second handle
     */
    quinticSpline::quinticSpline(std::vector<std::vector<Point>> points)
    {
        //Figuring our how many quintic bezier curves to generate
        double curveAmount = points.size() - 1;

        //Setting first curve manually

        curves.at(0) = quinticBezier(points.at(0).at(0), points.at(0).at(1), points.at(0).at(2), points.at(1).at(2), points.at(1).at(1), points.at(1).at(0));

        //Going over each point group, creating a curve for each group(except last)
        for(int pointGroupIndex = 1; pointGroupIndex <= curveAmount - 1; pointGroupIndex++)
        {
            //Calculating the handles used for generating the curve
            Point firstCalculatedHandle= (points.at(pointGroupIndex).at(0) * 2) - points.at(pointGroupIndex).at(1);
            Point secondCalculatedHandle = (points.at(pointGroupIndex).at(0) * 4) - (points.at(pointGroupIndex).at(1) * 4) + points.at(pointGroupIndex).at(2);

            //Creating curve object
            quinticBezier calculatedCurve = quinticBezier(points.at(pointGroupIndex).at(0), firstCalculatedHandle, secondCalculatedHandle, 
            points.at(pointGroupIndex + 1).at(2), points.at(pointGroupIndex + 1).at(1), points.at(pointGroupIndex + 1).at(0));

            //Adding curve onto the end of the curves vector
            curves.push_back(calculatedCurve);
        }
    }

    /**
     * @param u Interpolating value. The whole number is the curve to interpolate(-1), the decimal is the t value. Ex: u = 1.5, choosing second curve with t value of 0.5
     */
    Point quinticSpline::getPoint(double u)
    {
        int curveIndex = floor(u);
        double t = u - curveIndex;
        Point point = curves.at(curveIndex).getPoint(t);

        return point;
    }

    /**
     * @param u Interpolating value. The whole number is the curve to interpolate(-1), the decimal is the t value. Ex: u = 1.5, choosing second curve with t value of 0.5
     */
    Point quinticSpline::getFirstDerivative(double u)
    {
        int curveIndex = floor(u);
        double t = u - curveIndex;
        Point firstDerivative = curves.at(curveIndex).getFirstDerivative(t);

        return firstDerivative;
    }

    /**
     * @param u Interpolating value. The whole number is the curve to interpolate(-1), the decimal is the t value. Ex: u = 1.5, choosing second curve with t value of 0.5
     */
    Point quinticSpline::getSecondDerivative(double u)
    {
        int curveIndex = floor(u);
        double t = u - curveIndex;
        Point secondDerivative = curves.at(curveIndex).getSecondDerivative(t);

        return secondDerivative;
    }

    /**
     * @param u Interpolating value. The whole number is the curve to interpolate(-1), the decimal is the t value. Ex: u = 1.5, choosing second curve with t value of 0.5
     */
    double quinticSpline::getCurvature(double u)
    {
        int curveIndex = floor(u);
        double t = u - curveIndex;
        double curvature = curves.at(curveIndex).getCurvature(t);

        return curvature;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Waypoint struct
    waypoint::waypoint()
    {
        this->pose = Pose(0, 0, 0, 0);
        this->linVel = 0;
        this->angVel = 0;
        this->u = 0;
    }

    waypoint::waypoint(Pose pose, double linVel, double angVel, double u, double time)
    {
        this->pose = pose;
        this->linVel = linVel;
        this->angVel = angVel;
        this->u = u;
        this->time = time;
    }

//Trajectory struct

    trajectory::trajectory(std::vector<waypoint> points, double ds)
    {
        this->points = points;
        this->ds = ds;
    }


    trajectory::trajectory(std::vector<std::pair<Pose, std::vector<double>>> points, double ds)
    {   
        this->points.at(0) = waypoint(points.at(0).first, points.at(0).second.at(0), points.at(0).second.at(1), points.at(0).second.at(2), points.at(0).second.at(3));

        for(int i = 1; i < points.size(); i++)
        {
            this->points.push_back(waypoint(points.at(i).first, points.at(i).second.at(0), points.at(i).second.at(1), points.at(i).second.at(2), points.at(i).second.at(3)));
        } 

        this->ds = ds;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    profile::profile(double maxVel, double maxAccel, double maxDeccel)
    {
        this->maxVel = maxVel;
        this->maxAccel = maxAccel;
        this->maxDeccel = maxDeccel;
    }

    std::vector<std::vector<double>> profile::generateProfile(cubicBezier curve, double pointAmount, double k)
    {
        //Getting Points
        double deltaT = 1 / pointAmount;
        std::vector<std::vector<double>> points;
        double t = 0;

        while(t < 1)
        {   
            Point point = curve.getPoint(t);
            points.push_back({point.x, point.y});
            t += deltaT;
        }
        Point point = curve.getPoint(1);
        points.push_back({point.x, point.y});

        //Getting approximate arc length for each point, accuracy depends on amount of points
        double arcLength = 0;
        Point prevPoint = Point(points.at(0).at(0), points.at(0).at(1));
        points.at(0).push_back(arcLength);

        for(int i = 1; i < points.size(); i++)
        {
            Point point = Point(points.at(i).at(0), points.at(i).at(1));
            arcLength += pointToPointDistance(prevPoint, point);
            points.at(i).push_back(arcLength);
            prevPoint = point;
        }

        //Calculating max velocity at each point
        std::vector<double> maxVelocities;

        for(int i = 0; i < points.size(); i++)
        {
            Point firstDerivative = curve.getFirstDerivative(i * deltaT);
            Point secondDerivative = curve.getSecondDerivative(i * deltaT);
            double curvature = (sqrt(pow(secondDerivative.x, 2) + pow(secondDerivative.y,2))) / (pow(firstDerivative.x, 2) + pow(firstDerivative.y, 2));
            maxVelocities.push_back(fmin(maxVel, k / curvature));
        }

        //Forward pass
        std::vector<double> forwardPass;
        double prevVel = 0;

        for(int i = 0; i < points.size() - 1; i++)
        {
            //Getting distance to next point
            double deltaDistance = points.at(i).at(2) - points.at(i + 1).at(2);
            
            //Calculating how long it will take to get to next point, based off of initial velocity and acceleration
            double time_1 = (-prevVel + sqrt(pow(prevVel, 2) - 2 * maxAccel * (deltaDistance))) / maxAccel;
            double time_2 = (-prevVel - sqrt(pow(prevVel, 2) - 2 * maxAccel * (deltaDistance))) / maxAccel;
            //Taking appropriate root
            double time = fmax(time_1, time_2);
            
            //Calculating end velocity based off of the time it would take
            double vel = prevVel + maxAccel * time;
            //If end velocity is bigger than max velocity, then set it to max velocity
            vel = vel > maxVelocities.at(i) ? maxVelocities.at(i) : vel;
            
            //Add velocity to list
            forwardPass.push_back(vel);
            //Update prev velocity
            prevVel = vel;
        }
        //Setting end point vel manually becuse there is no point after it.
        forwardPass.push_back(maxVelocities.at(maxVelocities.size() - 1));

        //Backwards pass
        std::vector<double> backwardsPass = forwardPass; //Making sure its the right size so it gets no vector errors
        prevVel = 0;
        backwardsPass.at(points.size() - 1) = 0; //Setting first point seperately to ensure we end on 0 velocity. Cant start at 0, but should end at 0

        for(int i = points.size() - 2; i > 0; i--)
        {
            //Getting distance to next point
            double deltaDistance = points.at(i - 1).at(2) - points.at(i).at(2);

            //Calculating how long it will take to get to next point, based off of initial velocity and acceleration
            double time_1 = (-prevVel + sqrt(pow(prevVel, 2) - 2 * maxDeccel * (deltaDistance))) / maxDeccel;
            double time_2 = (-prevVel - sqrt(pow(prevVel, 2) - 2 * maxDeccel * (deltaDistance))) / maxDeccel;
            //Taking appropriate root
            double time = fmax(time_1, time_2);

            //Calculating end velocity based off of the time it would take
            double vel = prevVel + maxDeccel * time;
            //If end velocity is bigger than max velocity, then set it to max velocity
            vel = vel > forwardPass.at(i) ? forwardPass.at(i) : vel;

            //Add velocity to list
            backwardsPass.at(i) = vel;
            //Update prev velocity
            prevVel = vel;
        }
        //Setting last point manually againg because ther is no point after it
        backwardsPass.at(0) = forwardPass.at(0);


        //Generating output, with x, y, and vel, and t value(for interpolation)
        std::vector<std::vector<double>> output;

        for(int i = 0; i < backwardsPass.size(); i++)
        {
            output.push_back({points.at(i).at(0), points.at(i).at(1), backwardsPass.at(i), i * deltaT});
        }

        return output;

    }

    trajectory profile::generateProfile(quinticSpline spline, double ds, double k)
    {
        //dt = ds / sqrt(pow(deriv.x, 2) + pow(deriv.y, 2))
        //Delta t based off delta distance, derived from current t
        //From cooper motion profiling
        //Note: Only an approximation, more accurate with lower curvature, and lower ds

        //Forward pass

        double startVel = 0;

            std::vector<std::vector<double>> forwardPass = {{}};

            double u = 0;
            //Caluclating Linear Velocity
            double maxLinVelocity = fmin(this->maxVel, fabs(k / spline.getCurvature(u)));
            //vf*vf = vi*vi + 2ad
            double linVel = fmin(maxLinVelocity, sqrt((startVel * startVel) + 2 * maxAccel * ds));

            //Storing Lin velocity and u value
            forwardPass.at(0) = {linVel, u};
            double prevVel = linVel;

            //Getting delta u value based on delta distance 
                Point derivative = spline.getFirstDerivative(u);
                u += ds / sqrt(pow(derivative.x, 2) + pow(derivative.y, 2));

            while(u < spline.curves.size())
            {
                //Calculating lin vel
                    maxLinVelocity = fmin(maxVel, fabs(k / spline.getCurvature(u)));
                    linVel = fmin(maxLinVelocity, sqrt((prevVel * prevVel) + 2 * maxAccel * ds));

                //Storing velocities, time, and u value
                    forwardPass.push_back({linVel, u});

                //Updating u and prevVel
                    prevVel = linVel;
                    derivative = spline.getFirstDerivative(u);
                    u += ds / sqrt(pow(derivative.x, 2) + pow(derivative.y, 2));
            }


        //Backwards pass

            double endVel = 0;
            
            std::vector<std::vector<double>> backwardPass = forwardPass;

            u = forwardPass.at(forwardPass.size() - 1).at(1);
            
            //Caluclating Linear Velocity
                maxLinVelocity = forwardPass.at(forwardPass.size() - 1).at(0);
            //kinematic vfvi2ad
                linVel = fmin(maxLinVelocity, sqrt((endVel * endVel) + 2 * maxDeccel * ds));
            
            //Caluclating ang vel
                double angVel = linVel * spline.getCurvature(u);

            //Storing Velocities, time taken between points, and u value
                backwardPass.at(backwardPass.size() - 1) = {linVel, angVel, u};

                prevVel = linVel;

            //For loop because we know the number of points now
            for(int i = forwardPass.size() - 1; i--; i >= 0)
            {
                //Getting u
                    u = forwardPass.at(i).at(1);

                //Lin Vel
                    maxLinVelocity = forwardPass.at(i).at(0);
                    linVel = std::min(maxLinVelocity, sqrt((prevVel * prevVel) + 2 * maxAccel * ds));

                //Ang vel, reversed for cw+, converting to from rad/s to cm/s for path following
                    angVel = (linVel * spline.getCurvature(u)) * -1;

                //Saving
                    backwardPass.at(i) = {linVel, angVel, u};

                prevVel = linVel;
            }

        //Storing velocities and pose
        //Doesnt add time in yet

        Point position = spline.getPoint(backwardPass.at(0).at(2));
        derivative = spline.getFirstDerivative(backwardPass.at(0).at(2));
        double heading = atan3(derivative.y, derivative.x);
        double time = ds / backwardPass.at(0).at(0); //distance over velocity = time

        
        std::vector<waypoint> points = {waypoint(Pose(position.x, position.y, heading), backwardPass.at(0).at(0), backwardPass.at(0).at(1), backwardPass.at(0).at(2), time)};
        
        for(int i = 1; i < backwardPass.size(); i++)
        {
            position = spline.getPoint(backwardPass.at(i).at(2));
            derivative = spline.getFirstDerivative(backwardPass.at(i).at(2));
            heading = atan3(derivative.y, derivative.x);
            time = ds / backwardPass.at(i).at(0); //distance over velocity = time

            points.push_back(waypoint(Pose(position.x, position.y, heading), backwardPass.at(i).at(0), backwardPass.at(i).at(1), backwardPass.at(i).at(2), time));

            printf("Pos:(%.2f, %.2f), Lin:%.3f, Ang:%f, U:%.3f, I:%d, Time:%f\n", position.x, position.y, points.at(i).linVel, points.at(i).angVel, points.at(i).u, i, time);
        }



        return trajectory(points, ds);
        
    }