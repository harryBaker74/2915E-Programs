#include "main.h"
#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/boomerang.hpp"
#include "harryLibHeader/pathGen.hpp"
#include "harryLibHeader/velocityController.hpp"
#include <vector>
#include "harryLibHeader/globals.h"

namespace subsystems
{  
    void drivetrain::tangentIntersection(cubicBezier curve, bool async)
    {

        //leftDriveMotors.set_brake_mode_all(MOTOR_BRAKE_HOLD);
        //rightDriveMotors.set_brake_mode_all(MOTOR_BRAKE_HOLD);

        //Generating motion profile
        profile motionProfile(175.6, 473.4, 75);
        std::vector<std::vector<double>> profile = motionProfile.generateProfile(curve, 50, 1.90);

        vController::vController linCont (false);

        PID::PID angPID = PID::PID
        (
            10000,  //Kp
            0,  //Ki
            50000,  //Kd
            0,  //Windup Range
            0   //Max Integral
        );

        double exitShitass = 0.995;
        double correctionAmount = 5.0;
        double angSwitch = 10;

        double closestTValue = 0.0;
        double prevClosestTValue = 0.0;

        Point endingPoint = curve.getPoint(1); //make = to curve at t = 1
        Point nearEndPoint = curve.getPoint(0.99);

        double prevVel = 0;

        pros::delay(10);
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
                //Angular speed calculations
                    Point deltaPos(carrot.x - pose.x, carrot.y - pose.y);

                    double distance = pointToPointDistance(deltaPos, Point(0, 0));

                    double targetHeading = atan3(deltaPos.y, deltaPos.x);
                    double endingHeading = atan3(endTangent.y, endTangent.x); 

                    double distanceError = pointToPointDistance(pose, carrot);
                    double endDistanceError = pointToPointDistance(pose, endingPoint);


                    double endingRotation = pose.rotation + boundAngle(endingHeading - pose.heading, true);
                    double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);

                    double actualTargetRotation = getWeightedAverage(targetRotation, endingRotation, pow(fabs(distance), 3) / (pow(fabs(distance), 3) + pow(angSwitch, 3)));

                    double angOutput = angPID.getPid(pose.rotation, actualTargetRotation);

                //Linear speed Calculations
                    double tIndex = ceil(closestTValue * profile.size()); //Taking velcoity of point in front of self
                    double linVel = profile.at(tIndex - 1).at(2);
                    double rpmVel = linVel / (DRIVE_WHEEL_DIAMETER * M_PI * 2.54) * 60 / DRIVE_GEAR_RATIO;

                    double currentVel = (leftFrontMotor.get_actual_velocity() + rightFrontMotor.get_actual_velocity()) / 2;
                    double linOutput = linCont.rpmVelToVoltage(currentVel, prevVel, rpmVel);
                    prevVel = currentVel;

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