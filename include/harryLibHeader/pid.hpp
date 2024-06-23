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
    PID(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, double windupRange = 0.0, double maxIntegral = 0.0);

    //Setters

    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setWindupRange(double windupRange);
    void setMaxIntegral(double maxIntegral);

    //Pid Calculations

    double getPid(double error);
    double getPid(double currentPosition, double targetPosition);

    //Getters
    double getError();
    double getIntegral();
    double getDervative();

};

}