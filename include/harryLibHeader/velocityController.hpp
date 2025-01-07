#include "pid.hpp"

namespace vController
{
    class vController
    {
        double Kv;
        double Ka;
        double Ks;

        bool runTuner = false;

        public:

        vController(bool Ks);

        double rpmVelToVoltage(double currentVelocity, double targetVelocity);

        void tuner();
    };
}