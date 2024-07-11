#pragma once
namespace PID {

class PID
{
    //Constants
    double Kp;
    double Ki;
    double Kd;
    double windupRange;
    double maxIntegral;

    //Shitass i forgot the name of these
    double integral;
    double derivative;
    double error;
    double prevError;

    public:
    PID(double Kp, double Ki, double Kd, double windupRange, double maxIntegral);

    //Setters

    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setWindupRange(double windupRange);
    void setMaxIntegral(double maxIntegral);

    //Pid Calculations

    double const getPid(double error);
    double const getPid(double currentPosition, double targetPosition);

    //Getters
    double const getError();
    double const getIntegral();
    double const getDervative();

    //Reset Pid For Future Uses
    void reset();
};

}