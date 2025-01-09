//Shitassing here and just doing 2d mp no pp



#include "harryLibHeader/robot.hpp"
#include "main.h"

namespace subsystems
{
    void drivetrain::followPath(trajectory path)
    {  
        int index = 0;
        double switchTime = pros::millis() + path.points.at(index).second.at(2);

        while(index < path.points.size())
        {
            //Switch to next point
            if(switchTime <= pros::millis())
            {
                index += 1;
                switchTime = pros::millis() + path.points.at(index).second.at(2);
            }

            //Follow velocities at point
            double leftVel = path.points.at(index).second.at(0) + path.points.at(index).second.at(1);
            double rightVel = path.points.at(index).second.at(0) - path.points.at(index).second.at(1);

            setVoltage(leftVel, rightVel);
            pros::delay(10);
        }
    }
}

