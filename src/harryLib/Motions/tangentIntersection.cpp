#include "main.h"
#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/boomerang.hpp"
#include "harryLibHeader/pathGen.hpp"
#include <vector>

namespace subsystems
{  
    void drivetrain::tangentIntersection(cubicBezier curve, bool async)
    {

        //Generating motion profile
        profile motionProfile(0.0f, 0.0f);
        std::vector<std::vector<double>> profile = motionProfile.generateProfile(curve, 50, 5);

        double exitShitass = 0.995;
        double correctionAmount = 5.0;
        double cornerSlowdown = 30; // Higher values, slower around corners
        double angSwitch = 8;

        double closestTValue = 0.0;
        double prevClosestTValue = 0.0;

        Point endingPoint = curve.getPoint(1); //make = to curve at t = 1
        Point nearEndPoint = curve.getPoint(0.975);

        bool switchTarget = false;

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
                    double magnitude = pow(localTangent.x, 2) + pow(localTangent.y, 2); //This is actually the magnitude squared, but its used like this anyway so might aswell not sqrt it
                    Point secondDerivative = curve.getSecondDerivative(closestTValue);
                    double curvature = (sqrt(pow(secondDerivative.x, 2) + pow(secondDerivative.y, 2))) / (magnitude);

                //Amount to rotate tangent line by
                    double theta = (correctionAmount * signedCrossTrackError) * curvature;


                //Rotating local tangent line
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
                double endingHeading = atan3(endingPoint.y - nearEndPoint.y, endingPoint.x - nearEndPoint.x); 

                double distanceError = pointToPointDistance(pose, carrot);
                double endDistanceError = pointToPointDistance(pose, endingPoint);


                double endingRotationError = boundAngle(endingHeading - pose.heading, true);
                double targetRotationError = boundAngle(targetHeading - pose.heading, true);

            

                double angOutput = angPID.getPid(targetRotationError);
                double linOutput = linPID.getPid(fmin(fmax(distanceError, endDistanceError), 50)) / fmax((fmax(futureCurvature, curvature) * cornerSlowdown), 1);
                Controller.print(0, 0, "%.4f", closestTValue);

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