#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/exitConditions.hpp"
#include "harryLibHeader/gainSchedular.hpp"
namespace subsystems
{

void drivetrain::turnToHeading(double heading, int timeout_ms, bool clockwise, bool radians, bool async)
{   
    //Prevent this motion from starting if the robot is already in a motion
    while(inMotion)
        pros::delay(10);
    
    //Run function in a task if it is supposed to be async
    if (async)
    {
        pros::Task task {[=, this] {
            turnToHeading(heading, timeout_ms, clockwise, radians, false);
            pros::Task::current().remove();
        }};
    }
    else
    {
        int endTime = pros::millis() + timeout_ms;
        this->distanceTraveled = 0;
        this->inMotion = true;

        //TUNING  
            //Guide to tuning
            //Setup
            //Set all pid variables to zero, and comment out the break in the exit condition if statement
            //Run this function whenever you change the constants and take the error value to graph(print it or something)
            //Give the function something like 90 degrees everytime you run so it actually has time to work
            //Alrtenatively, give it random targets each time so your constants work better over multiple different target ranges
            //
            //Pid Tuning
            //Start off by increasing Kp until you get 2-4 oscillations
            //Then increase kd until the oscilattions stop, the graph should look like one sloped line curving into a straight horizontal line
            //Set windup range to a bit above the current steady state error
            //Set max integral to whatever you want honestly you prob dont need it
            //Start increasing Ki very slowly until steady state error dissapears quickly.
            //
            //At this point un-comment the break in the exit condition if statement
            //
            //Exit Conditions Tuning
            //(Note these values are in rad, rad/s, and rad/(s*s), so convert from deg if needed)
            //Set the error exit to something like 0.1 degrees if your confident, or 0.25 if your not. Maybe go even lower if its really good
            //Change around vel exit and accel exit to see what works best.(Aut Student Jordan D'Souza, Passionate Learner)
            //In theory, vel exit should be about 1/100 error exit, and accel exit should be 1/10000 of error exit. but this might be wrong idk

            double angle = heading;
            if(!radians) angle *= M_PI / 180;  //Converting to radians if needed

            //Finding the target rotation
            //Gets the difference between the desired bounded heading and the current bounded heading, then bounds this angle, then gets the current rotation added to it
            //In a roundabout way because we dont want to do turns bigger than 180 deg/1 pi, and its easier to track because the values cant overflow due to using rotation
            // double difference = (boundAngle(angle - pose.heading, true));
            // double targetRotation = pose.rotation + difference;

            //Bounding to = -180 - 180
            angle = boundAngle(angle, true);

            //Setting target rotation based on direction
            if(clockwise)
            {
                //If angle is smaller than heading, sub diff from 360 to make it a clockwise turn
                //Ex, angle = 35, heading = 90, diff = 90-35 = 55
                //angle = 360 - (55) = 305
                //Ex, angle = -35, heading = 90, diff = 90 + 35 = 125
                //angle = 360 - 125 = 235
                if(angle < pose.heading)
                {
                    double diff = pose.heading - angle;
                    angle = (2*M_PI) - diff;
                }
                //If angle is bigger, then set=to diff
                //Ex, angle = 35, heading = -45, diff = 35 - -45 = 80
                //Ex, angle = 50, heading = 45, diff = 50 - 45 = 5
                //Ex, angle = -20, heading = -40, diff = -20 - -40 = 20
                else if(angle > pose.heading)
                {
                    double diff = angle - pose.heading;
                    angle = diff;
                }
            }
            else
            {
                //If angle is bigger than heading, subtract 360 from it to make it a counter clockwise turn
                //Ex angle = 90, heading = 35, diff = 90 - 35 = 55
                //angle = 55 - 360 = -305
                //Ex angle = 0, heading = -80, diff = 0 - -80 = 80
                //angle 80 - 360 = -280
                if(angle > pose.heading)
                {
                    double diff = angle - pose.heading;
                    angle = diff - (2*M_PI);
                }
                //If angle is smaller, then set=to difference
                //Ex, angle = -35, heading = 45, diff = -35 - 45 = -80
                //Ex, angle = 45, heading = 50, diff = 45 - 50 = -5
                //Ex, angle = -40, heading = -20, diff = -40 - -20 = -20
                else if(angle < pose.heading)
                {
                    double diff = angle - pose.heading;
                    angle = diff;
                }
            }

            //Adding angle to rotation to give an unbounded target to approach
            double targetRotation = pose.rotation + angle;


            PID::PID pid(
                350,
                0,
                3000,
                0,
                0
            );

            double maxTurnSpeed = 80;

            //Exit conditions
            double errorExit = 0.1;
            // double errorExit = 0.043;
            double velExit = 0.1;

            
        //Everything else
        double prevVel = 0;
        double prevRotation = this->pose.rotation;

        while(pros::millis() < endTime)
        {
            //Pid Voltage Calculations
            double output = pid.getPid(pose.rotation, targetRotation);

            output = std::clamp(output, -maxTurnSpeed, maxTurnSpeed);

            //Setting drivetrain voltage based on Pid Output and Slewing
            this->setVoltage(output / 127 * 12000, -output / 127 * 12000);

            //Exit Conditions
            //Error, and Velocity based(only exit when both are low) so we dont need to worry about waiting with low error for a certain amount of time
            //Because error is in rad: vel is rad/s
            double error = pid.getError();
            double errorVel = pid.getDervative() / 0.01; //Derivative is already just the rate of change

            //Checking if all these values are below a certain threshold
            if (exitConditions::rangeExit(error, errorExit) && exitConditions::rangeExit(errorVel, velExit))
            {
                break;
            }
            //Updating distance traveled for async functions
            this->distanceTraveled += (this->pose.rotation - prevRotation) * 180 / M_PI;

            Controller.print(0, 0, "%f", pid.getError() * 180 / M_PI);


            //Delay for scheduling
            pros::delay(10);
        }
        setVoltage(0, 0);
        this->inMotion = false;
    };
}

void drivetrain::turnToHeading(double heading, int timeout_ms, bool radians, bool async)
{
    turnToHeading(heading, timeout_ms, true, radians, async);
}

void drivetrain::turnToHeading(double heading, bool clockwise, int timeout_ms)
{
    turnToHeading(heading, timeout_ms, clockwise, false, false);
}

void drivetrain::turnToPoint(Point point, int timeout_ms, bool async)
{
    double angle = atan3(point.y - pose.y, point.x - pose.x);
    turnToHeading(angle, timeout_ms, false, true, async);
}

}