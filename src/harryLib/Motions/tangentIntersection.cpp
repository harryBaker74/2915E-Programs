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
    void drivetrain::tangentIntersection(cubicBezier curve, std::vector<std::vector<double>> profile, bool backwards, bool async)
    {

        leftDriveMotors.set_brake_mode_all(MOTOR_BRAKE_HOLD);
        rightDriveMotors.set_brake_mode_all(MOTOR_BRAKE_HOLD);

        vController::vController linCont (false);

        PID::PID angPID = PID::PID
        (
            20000,  //Kp
            0,  //Ki
            200000,  //Kd
            0,  //Windup Range
            0   //Max Integral
        );

        double exitShitass = 0.995;
        double correctionAmount = 0.2;

        double closestTValue = 0.0;
        double prevClosestTValue = 0.0;

        Point endingPoint = curve.getPoint(1); //make = to curve at t = 1

        double prevVel = 0;

        pros::delay(50);
        //Main loop
        while(true)
        {
            
            //Finding t value asscoiated with the point on the curve closest to the robot.
                closestTValue = curve.smallestDistance(Point(pose.x, pose.y), prevClosestTValue);
                
                if(closestTValue == 0)
                    closestTValue += 0.00001;

                prevClosestTValue = closestTValue;

            //Cross track error thing
                Point tangentVector = curve.getFirstDerivative(closestTValue);
                Point distanceVector = Point(pose.x, pose.y) - curve.getPoint(closestTValue);
                double signedCrossTrackError = pointToPointDistance(Point(pose.x, pose.y), curve.getPoint(closestTValue));
                signedCrossTrackError *= sign(tangentVector.cross(distanceVector));
            
            //Calculate tangents
                Point endTangent = curve.getFirstDerivative(1);
                double endSlope = endTangent.y / (endTangent.x + 0.000001);
                double endYIntercept = endingPoint.y - (endSlope * endingPoint.x);
                Point localTangent = curve.getFirstDerivative(closestTValue);
                Point closestPoint = curve.getPoint(closestTValue);
                double localSlope = localTangent.y / (localTangent.x + 0.000001);
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
                double carrotX = 0;
                double carrotY = 0;
                carrotX = (localYIntercept - endYIntercept) / (endSlope - localSlope);
                carrotY = (endSlope * carrotX) + endYIntercept;
                Point carrot = Point(carrotX, carrotY);


            //Move towards intersection point
                //Angular speed calculations
                    Point deltaPos(carrot.x - pose.x, carrot.y - pose.y);

                    double distance = pointToPointDistance(deltaPos, Point(0, 0));

                    double targetHeading = atan3(deltaPos.y, deltaPos.x);
                    double endingHeading = atan3(endTangent.y, endTangent.x); 

                    Controller.print(0, 0, "%.2f, %.2f", endTangent.x, endTangent.y);

                    if(backwards)
                    {
                        targetHeading = boundAngle(targetHeading + M_PI, true);
                        endingHeading = boundAngle(endingHeading + M_PI, true);
                    }

                    double endingRotation = pose.rotation + boundAngle(endingHeading - pose.heading, true);
                    double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);

                    double actualTargetRotation = getWeightedAverage(endingRotation, targetRotation, pow(closestTValue, 4));

                    double angOutput = angPID.getPid(pose.rotation, actualTargetRotation);

                //Linear speed Calculations
                    double tIndex = ceil(closestTValue * profile.size()); //Taking velcoity of point in front of self
                    double linVel = profile.at(tIndex - 1).at(2);
                    double rpmVel = linVel / (DRIVE_WHEEL_DIAMETER * M_PI * 2.54) * 60 / DRIVE_GEAR_RATIO;

                    double currentVel = (leftFrontMotor.get_actual_velocity() + rightFrontMotor.get_actual_velocity()) / 2;
                    
                    if(backwards)
                        rpmVel *= -1;

                    double linOutput = linCont.rpmVelToVoltage(currentVel, prevVel, rpmVel);
                    prevVel = currentVel;

                //Wheel voltage calculations
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


                //Exit conditions
                if((closestTValue >= exitShitass) && (fabs(angPID.getError()) < 0.01))
                {
                    //Exit if Robot is close to the end of the curve
                    break;
                }

            //Delay for other tasks
            pros::delay(10);
        }
    }
}