#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/exitConditions.hpp"

namespace subsystems
{
    void drivetrain::swingToHeading(double heading, bool side, bool backwards, bool radians, bool async, int timeout_ms)
    {
        //Prevent multiple motions from running at once
        while (inMotion)
            pros::delay(10);
        if (async)
        {
            pros::Task task {[=, this]{
                swingToHeading(heading, side, backwards, radians, false);
                pros::Task::current().remove();
            }};
        }
        else
        {
            this->distanceTraveled = 0;
            this->inMotion = true;

            PID::PID pid = PID::PID
            {
                40000,
                0,
                75000,
                0,
                0
            };

            double errorExit = 0.01;
            double velExit = 1;

            double minSpeed = 0;
            
            double angle = heading;
            if(!radians)
                angle *= M_PI/180;
            
            //Calculating target heading
            double targetRotation;
            double headingDifference = (boundAngle(angle - pose.heading, true));
            
            //Cases where heading increases
            //left forward, right backward
            //side = true, backwards = false
            //side = false, backwards = true
            //xor
            
            //Cases where heading decreases
            //left backwards, right forwards
            //side = true, backwards = true
            //side = false, backwards = false
            //and
            
            if(side == backwards)//Keeps targetRotation < pose.rotation for negative swings
            {
                if(headingDifference <= 0)
                    targetRotation = pose.rotation + headingDifference;
                else
                    targetRotation = pose.rotation + headingDifference - (2 * M_PI);
            }
            else//Keeps targetRotation > pose.rotation for positive swings
            {
                if(headingDifference >= 0)
                    targetRotation = pose.rotation + headingDifference;
                else
                    targetRotation = pose.rotation + headingDifference + (2 * M_PI);
            }

            int endTime = pros::millis() + timeout_ms;

            while(pros::millis() <= endTime)
            {
                //Getting error
                double rotationDifference = targetRotation - pose.rotation;

                //Getting pid output
                double output = pid.getPid(rotationDifference);

                if(fabs(output) < minSpeed)
                {
                    output = sign(output) * minSpeed;
                }

                //Setting motor voltage based on output, and which side should be moving
                setVoltage(output * side, output * (side - 1));
            
                //Exit conditions, velocity + range exit
                if(exitConditions::rangeExit(pid.getError(), errorExit) && exitConditions::rangeExit(pid.getDervative(), velExit))
                {
                    break;
                }

                // printf("Output:%.2f, Error:%.2f\n", output, pid.getError());

                //Updating async variables
                this->distanceTraveled += fabs(pose.rotation - prevPose.rotation);

                pros::delay(10);
            }
            this->inMotion = false;
        }
    
    }

}