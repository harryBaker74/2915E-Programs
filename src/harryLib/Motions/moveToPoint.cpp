#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/exitConditions.hpp"
#include "harryLibHeader/gainSchedular.hpp"

namespace subsystems
{
    void drivetrain::moveToPoint(Point point, bool backwards, int timeout_ms, bool async)
    {
        //Prevent this motion from starting if the robot is already in a motion
        while(inMotion)
            pros::delay(10);


        if(async)
        {
            pros::Task task {[=, this] {
                moveToPoint(point, backwards, timeout_ms, false);
                pros::Task::current().remove();
            }};
        }
        else
        {
            this->distanceTraveled = 0;
            this->inMotion = true;

            PID::PID angPid = PID::PID(
                700,
                0,
                6000,
                0,
                0
            );
            PID::PID linPid = PID::PID(
                10,  //Kp
                0.3,  //Ki
                80,  //Kd
                2,  //Windup Range
                0   //Max Integral
            );

            //Exit conditions
            double errorExit = 2;
            double velExit = 0.01;

            //Angular Falloff Parameter
            //Smaller number means more abrupt falloff, with 0 being no fall off
            //Angular speed will be half when distance is at angK.
            //Eg:   angK = 1, angVel *= 0.5 for hypot = 1;
            //      angK = 2, angVel *= 0.5 for hypot = 2;
            //      angK = 3: angVel *= 0.5 for hypot = 3;
            //      etc.
            double angK = 25;

            int linMultiplier = backwards ? -1 : 1;
            // bool close = false;
            // double endRotation = 0;

            //Timeout
            int endTime = pros::millis() + timeout_ms;
            while(pros::millis() < endTime)
            {

                //Calculating delta cartesian offesets
                double deltaX = point.x - pose.x;
                double deltaY = point.y - pose.y;

                //Converts delta cartesian coordinates to polar coordinates, than takes theta and adds pi/2 to it to convert it to +y = 0, then bounds the angle;
                double targetHeading = atan3(deltaY, deltaX);

                targetHeading = backwards ? boundAngle(targetHeading + M_PI, true) : targetHeading;

                double targetRotation = pose.rotation + boundAngle(targetHeading - pose.heading, true);

                //Calculating distance to point
                double hypot = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

                //If getting close to point
                // if((hypot < (7.5 * 2.54)) && !close)
                // {
                //     close = true;
                // }

                //Calculating angular velocity multiplier
                //Makes angular velocity weaker the closer you are to the point, eventually becoming zero when on the point
                //Stops spinning in circles, idea from Genesis/Daniel
                // double angMultiplier = fabs(hypot) / (fabs(hypot) + angK);
                //Same thing but with a different function, has a quicker drop off
                double angMultiplier = pow(-2, -(fabs(hypot*hypot)/angK)) + 1;

                

                //Interpolating between angle to point and ending angle as you get closer to the point, prevents spinning in circles
                // if(close)
                //     targetRotation = getWeightedAverage(targetRotation, endRotation, pow(-2, -(fabs(hypot*hypot)/angK)) + 1);

                //Calculating voltage
                double angOutput = angPid.getPid(pose.rotation, targetRotation) * angMultiplier;
                // double linOutput = std::fmin(linPid.getPid(hypot), 12000 * cos(std::fmin(fabs(targetRotation - this->pose.rotation), 90))) * linMultiplier;
                double linOutput = linPid.getPid(hypot) * cos(fabs(targetRotation - this->pose.rotation)) * linMultiplier;

                //Preventing over saturation, prioritising turning over driving
                if((fabs(linOutput) + fabs(angOutput)) > 12000)
                    linOutput = (12000 - fabs(angOutput)) * linMultiplier;
                
                //Powering motors
                double leftVoltage = linOutput + angOutput;
                double rightVoltage = linOutput - angOutput;
                //Scaling to -12000 -> 12000
                this->setVoltage(leftVoltage / 127 * 12000, rightVoltage / 127 * 12000);


                // double adjustedAngle = boundAngle(M_PI_2 - endTheta, true);
                // bool robotSide = (pose.y - point.y) * -sin(adjustedAngle) <= (pose.x - point.x) * cos(adjustedAngle) + earlyExitRange;

                //Exit Conditions, Semi circle exit
                //Should add velocity exit here in the future
                if(exitConditions::rangeExit(hypot, errorExit))
                    break;

                Controller.print(0, 0, "%.2f, %.2f", pose.x, pose.y);

                //Updating distance traveled for async functions
                this->distanceTraveled += sqrt(pow(this->pose.x - this->prevPose.x, 2) + pow(this->pose.y - this->prevPose.y, 2));

                //Delay for other tasks
                pros::delay(10);
            }
            this->inMotion = false;
        }
    }
}