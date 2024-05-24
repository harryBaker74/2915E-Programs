#include "../../include/main.h"

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
    PID(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, double windupRange = 0.0, double maxIntegral = 0.0)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->windupRange = windupRange;
        this->maxIntegral = maxIntegral;
    }

    //Setters

    void setKp(double Kp)
    {this->Kp = Kp;}
    void setKi(double Ki)
    {this->Ki = Ki;}
    void setKd(double Kd)
    {this->Kd = Kd;}
    void setWindupRange(double windupRange)
    {this->windupRange = windupRange;}
    void setMaxIntegral(double maxIntegral)
    {this->maxIntegral = maxIntegral;}

    //Pid Calculations

    double getPid(double error)
    {
        //Setting data variables
        this->error = error;
        this->derivative = this->error - prevError;
        this->integral += this->error;

        //Anti windup
            //Windup Range
            if(this->error < fabs(this->windupRange))
                this->integral = 0;
            //Clamping
            if(fabs(this->integral) > fabs(this->maxIntegral))
                this->integral = 0;

            //Sign-Flip Reset
            if(sign(prevError) != sign(error))
                this->integral = 0;
        
        return (Kp * this->error) + (Ki * this->integral) + (Kd * derivative);

    }
    double getPid(double currentPosition, double targetPosition)
    {
        return getPid(targetPosition - currentPosition);
    }

    //Getters
    double getError() {return this->error;}
    double getIntegral() {return this->integral;}
    double getDervative() {return this->derivative;}

};

}