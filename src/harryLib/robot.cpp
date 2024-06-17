#include "../include/main.h"
#include "../include/harryLibHeader/devices.h"

//File for controlling all systems in the robot

namespace drivtrain
{

    class drivetrain
    {   

        public:
        //constructor
        drivetrain(){};

        
        void setDriveMotors(int left, int right)
        {
            
        }

        void driverFunctions()
        {

        }

    };

    //Pose struct
    struct pose
    {
        double x;
        double y;
        double heading;

        public:
        pose(double x, double y, double heading)
        {
            this->x = x;
            this->y = y;
            this->heading = heading;
        }

        void set(double x, double y, double heading)
        {
            this->x = x;
            this->y = y;
            this->heading = heading;
        }
    };

    double distanceTravelled;
    bool moving;

    void moveToPose()
    {
        
    }

}

namespace subsystems
{

}