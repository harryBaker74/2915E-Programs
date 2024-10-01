#include "main.h"
#include "harryLibHeader/robot.hpp"

namespace subsystems
{  
    void drivetrain::tangentIntersection(cubicBezier curve, bool async = true)
    {
        double closestTValue = 0.0;
        double prevClosestTValue = 0.0;

        Point endingPoint = curve; //make = to curve at t = 1

        //Main loop
        while(true)
        {

            closestTValue = 0; //Set to current closest t value
            prevClosestTValue = closestTValue;

            //Cross track error thing here

            //Calculate tangents

            //Rotate local tangent based on cross track error

            //Move towards intersection point

            
        }
    }
}