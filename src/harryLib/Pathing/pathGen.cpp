#include "harryLibHeader/pathGen.hpp"
#include "harryLibHeader/util.hpp"
#include "main.h"

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