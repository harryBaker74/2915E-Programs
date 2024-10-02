#include "main.h"
#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"

namespace subsystems
{  
    void drivetrain::tangentIntersection(cubicBezier curve, bool async = true)
    {

        PID::PID angPID = PID::PID
        (
            0,  //Kp
            0,  //Ki
            0,  //Kd
            0,  //Windup Range
            0   //Max Integral
        );

        PID::PID linPID = PID::PID
        (
            0,  //Kp
            0,  //Ki
            0,  //Kd
            0,  //Windup Range
            0   //Max Integral
        );

        double exitShitass = 0.985;
        double correctionAmount = 0;

        double closestTValue = 0.0;
        double prevClosestTValue = 0.0;

        Point endingPoint = curve.getPoint(1); //make = to curve at t = 1



        //Main loop
        while(true)
        {
            
            //Finding t value asscoiated with the point on the curve closest to the robot.
            closestTValue = curve.smallestDistance(Point(pose.x, pose.y), prevClosestTValue);


            //Exit conditions
            if(closestTValue >= exitShitass)
            {
                //Exit if Robot is close to the end of the curve
                break;
            }

            prevClosestTValue = closestTValue;

            //Cross track error thing
            Point tangentVector = curve.getFirstDerivative(closestTValue);
            Point distanceVector = Point(pose.x, pose.y) - curve.getPoint(closestTValue);
            double signedCrossTrackError = pointToPointDistance(Point(pose.x, pose.y), curve.getPoint(closestTValue));
            signedCrossTrackError *= sign(tangentVector.cross(distanceVector));
            
            //Calculate tangents
            Point endTangent = curve.getFirstDerivative(1);
            double endSlope = endTangent.y / endTangent.x;
            double endYIntercept = endingPoint.y - (endSlope * endingPoint.x);
            Point localTangent = curve.getFirstDerivative(closestTValue);
            Point closestPoint = curve.getPoint(closestTValue);
            double localSlope = localTangent.y / localTangent.x;
            double localYIntercept = closestPoint.y - (localSlope * closestPoint.x);

            //Rotate local tangent based on cross track and curvature
            //Figuring out curvature, math at the bottom of https://www.desmos.com/calculator/8v04tzgocu
            double magnitude = sqrt(pow(localTangent.x, 2) + pow(localTangent.y, 2));
            Point secondDerivative = curve.getSecondDerivative(closestTValue);
            Point unitTangentVec = Point(secondDerivative.x / pow(magnitude, 2), secondDerivative.y / pow(magnitude, 2));
            double curvature = sqrt(pow(unitTangentVec.x, 2) + pow(unitTangentVec.y, 2)); 

            //Amount to rotate tangent line by
            double theta = (correctionAmount * signedCrossTrackError) * curvature;


            //Rotating tangent line
            //The math for this is at https://www.desmos.com/calculator/jdpegpuz7e
            double A = -localSlope;
            double B = 1;
            double C = -localYIntercept;
            double x = closestPoint.x;
            double y= closestPoint.y;
            localSlope =   (((-A * cos(theta)) - (B * sin(theta))) / 
                            ((B * cos(theta)) - (A * sin(theta))));
            localYIntercept = (((A * x * (cos(theta) - 1)) + (B * (x * sin(theta) - y)) - C)/
                                ((B * cos(theta)) - (A * sin(theta)))) + y;

            //Get Intersection Point
            double carrotX = (localYIntercept - endYIntercept) / (endSlope - localSlope);
            double carrotY = (endSlope * carrotX) + endYIntercept;
            Point carrot = Point(carrotX, carrotY);


            //Move towards intersection point
            Point deltaPos(carrot.x - pose.x, carrot.y - pose.y);

            double targetHeading = atan3(deltaPos.y, deltaPos.x);
            double targetRotationError = boundAngle(targetHeading - pose.heading, true);
            double distanceError = pointToPointDistance(Point(pose.x, pose.y), carrot);

            double angOutput = angPID.getPid(targetRotationError);
            double linOutput = linPID.getPid(distanceError);

            double leftOuput = linOutput + angOutput;
            double rightOutput = linOutput - angOutput;

            //Preventing oversaturation proportionally, taken from lemlib
            double ratio = fmax(fabs(leftOuput), fabs(rightOutput)) / 12000;
            if(ratio > 1)
            {
                leftOuput /= ratio;
                rightOutput /= ratio;
            }

            setVoltage(leftOuput, rightOutput, true, 10);

            //Delay for other tasks
            pros::delay(10);
        }
    }
}