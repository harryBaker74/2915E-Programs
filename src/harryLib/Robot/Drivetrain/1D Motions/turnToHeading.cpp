#include "harryLibHeader/robot.hpp"
#include "harryLibHeader/pid.hpp"
#include "harryLibHeader/velocityController.hpp"
#include "harryLibHeader/exitConditions.hpp"
namespace subsystems
{

void drivetrain::turnToHeading(double heading, int timeout_ms, bool radians, bool async)
{   
    //Prevent this motion from starting if the robot is already in a motion
    while(inMotion)
        pros::delay(10);
    
    //Run function in a task if it is supposed to be async
    if (async)
    {
        pros::Task task {[=, this] {
            turnToHeading(heading, radians, false);
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
            //Change around vel exit and accel exit to see what works best.(Aut Student Jordan D'Souza)
            //In theory, vel exit should be about 1/100 error exit, and accel exit should be 1/10000 of error exit. but this might be wrong idk

            //Pid for turning 
            PID::PID pid(
                600.0,    //Kp
                120.0,    //Ki
                6500.0,    //Kd
                0.07,    //Windup Range
                0.0     //Max Intergal
            );

            //Exit conditions
            double errorExit = 0.015;
            double velExit = 0.008;

            
        //Everything else
        double angle = heading;
        double prevVel = 0;
        int counter = 0;
        
        if(!radians) angle *= M_PI / 180;  //Converting to radians if needed

        //Finding the target rotation
        //Gets the difference between the desired bounded heading and the current bounded heading, then bounds this angle, then gets the current rotation added to it
        //In a roundabout way because we dont want to do turns bigger than 180 deg/1 pi, and its easier to track because the values cant overflow due to using rotation
        double targetRotation = pose.rotation + (boundAngle(angle - pose.heading, true));

        while(pros::millis() < endTime)
        {
            //Pid Voltage Calculations
            double output = pid.getPid(pose.rotation, targetRotation);
            
            //Setting drivetrain voltage based on Pid Output and Slewing
            this->setVoltage(output, -output, true, 10);

            //Exit Conditions
            //Error, and Velocity based(only exit when both are low) so we dont need to worry about waiting with low error for a certain amount of time
            //Because error is in rad: vel is rad/s
            double error = pid.getError();
            double errorVel = pid.getDervative() / 0.01; //Derivative is already just the rate of change

            //Checking if all these values are below a certain threshold
            if (exitConditions::rangeExit(error, errorExit) && exitConditions::rangeExit(errorVel, velExit))
                break;

            //Updating distance traveled for async functions
            this->distanceTraveled += this->pose.rotation - this->prevPose.rotation;
            
            //Delay for scheduling
            pros::delay(10);
        }
        this->inMotion = false;
    };
}

}