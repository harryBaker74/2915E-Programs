#pragma once

//File for controlling all systems in the robot

namespace drivetrain
{

    class drivetrain
    {   

        public:
        //constructor
        drivetrain();

        /**
         * @brief Function to set the voltage applied to all drive motors
         * 
         * @param left Voltage to left side of the drive(-12000 to 12000)
         * @param right Voltage to right side of the drive(-12000 to 12000)
         */
        void setDriveMotors(int left, int right);

        /**
         * @brief Function that handles controller inputs for drivetrain. Intended to be used during Op Control
         * 
         */
        void driverFunctions();

    };

    //Pose struct
    struct pose
    {
        double x;
        double y;
        double heading;

        public:
        pose(double x, double y, double heading);

        void set(double x, double y, double heading);
    };


    void moveToPose();

}

namespace subsystems
{
    class intake
    {
        //Constructor
        intake();

        //Function to set intake voltage
        void setIntakeMotors(int voltage);

        //Function to run intake during driver comtrol
        void driverFunctions
    };
}