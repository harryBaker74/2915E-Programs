#include "../include/main.h"
#include "../include/harryLibHeader/robot.hpp"

//File for controlling all systems in the robot

namespace drivetrain
{
        //constructor
        drivetrain::drivetrain(){};

        /**
         * @brief Function to set the voltage applied to all drive motors
         * 
         * @param left Voltage to left side of the drive(-12000 - 12000)
         * @param right Voltage to right side of the drive(-12000 - 12000)
         */
        void drivetrain::setDriveMotors(int left, int right)
        {
            //Setting Left Motors
            leftFront.move_voltage(left);
            leftMid.move_voltage(left);
            leftBack.move_voltage(left);

            //Setting Right Motors
            rightFront.move_voltage(right);
            rightMid.move_voltage(right);
            rightBack.move_voltage(right);
        }

        /**
         * @brief Function that handles controller inputs for drivetrain. Intended to be used during Op Control
         * 
         */
        void drivetrain::driverFunctions()
        {
            int leftJoystick = Controller.get_analog(ANALOG_LEFT_Y);
            int rightJoystick = Controller.get_analog(ANALOG_RIGHT_Y);

            int leftOutput = floor(linearToCubed(leftJoystick, 127, 1));
            int rightOutput = floor(linearToCubed(rightJoystick, 127, 1));

            setDriveMotors(leftOutput, rightOutput);
        }

    //Pose struct

    
        pose::pose(double x, double y, double heading)
        {
            this->x = x;
            this->y = y;
            this->heading = heading;
        }

        void pose::set(double x, double y, double heading)
        {
            this->x = x;
            this->y = y;
            this->heading = heading;
        }

    double distanceTravelled;
    bool moving;

    void moveToPose()
    {
        
    }

}

namespace subsystems
{

}