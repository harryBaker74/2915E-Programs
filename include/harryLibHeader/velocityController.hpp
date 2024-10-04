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

        double rpmVelToVoltage(double currentVelocity, double prevVelocity, double targetVelocity);

        void startTuner(double velocity);

        void endTuner();
    };
}