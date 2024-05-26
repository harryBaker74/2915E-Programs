#include "../include/main.h"

namespace drivtrain
{

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