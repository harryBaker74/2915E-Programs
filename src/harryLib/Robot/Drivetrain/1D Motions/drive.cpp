#include "harryLibHeader/robot.hpp"

namespace subsystems
{
    //Should be changed to a real 1 dimensional controller
    void drivetrain::drive(double distance, bool async)
    {
        double startHeading = pose.heading;

        this->moveToPoint(Point(pose.x + (distance * sin(pose.heading)), pose.y + (distance * cos(pose.heading))), distance < 0, async);
    }
}