#include "pid.hpp"

namespace vController
{
    class vController
    {

        bool runTuner = false;

        //Pid for converting wheel rpm to motor voltage
        PID::PID rpmPid = PID::PID(
            0.0,    //Kp
            0.0,    //Ki
            0.0,    //Kd
            0.0,    //Windup Range
            0.0     //Max Intergal
        );

        public:

        vController();  

        double rpmVelToVoltage(double currentVelocity, double targetVelocity);

        void reset();

        void startTuner(double velocity);

        void endTuner();
    };
}