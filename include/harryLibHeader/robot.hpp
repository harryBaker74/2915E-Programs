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
        void setVoltage(double left, double right);

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
        //Constructor
        pose(double x, double y, double heading);

        /**
         * @brief Function to set this pose to a deisred pose
         * 
         * @param x What to set the x value
         * @param y What to set the y value
         * @param heading What to set the heading
         */
        void set(double x, double y, double heading);

        /**
         * @brief Function to set this pose to a deisred pose
         * 
         * @param pose The pose to set this pose to
         */
        void set(pose pose);
    };


    /**
     * @brief Function to move the robot from its current pose to the desired pose
     * Uses a Boomerang controller
     */
    void moveToPose();
    /**
     * @brief Function to move the robot from its current pose to the desired pose
     * Uses an intemediate direction
     */
    void moveToPose();

    /**
     * @brief Function to make the robot follow a desired path from the path generator or path scheduler
     */
    void followPath();

}

namespace subsystems
{
    class intake
    {
        public:
        //Constructor
        intake();

        //Function to set intake voltage
        void setVoltage(double voltage);

        //Function to run intake during driver control
        void driverFunctions();
    };

    class plunger
    {
        public:
        //Constructor
        plunger();

        //Function to set plunger voltage
        void setVoltage(double voltage);

        //Function to run plunger during driver control
        void driverFunctions();
    };

    class mogo
    {
        int mogoPressCount = 0;

        public:
        //Constructor
        mogo();

        //Function to set mogo output
        void setState(bool state);

        //Function to run mogo during driver control
        
        void driverFunctions();
    };
}