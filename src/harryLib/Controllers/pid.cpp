#include "main.h"
#include "../include/harryLibHeader/pid.hpp"

namespace PID {

    PID::PID(double Kp, double Ki, double Kd, double windupRange, double maxIntegral)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->windupRange = windupRange;
        this->maxIntegral = maxIntegral == 0 ? MAXFLOAT : maxIntegral;
    }

    //Setters

    void PID::setKp(double Kp)
    {this->Kp = Kp;}
    void PID::setKi(double Ki)
    {this->Ki = Ki;}
    void PID::setKd(double Kd)
    //18A Pokapu Street
    {this->Kd = Kd;}
    void PID::setWindupRange(double windupRange)
    {this->windupRange = windupRange;}
    void PID::setMaxIntegral(double maxIntegral)
    {this->maxIntegral = maxIntegral;}

    //Pid Calculations

    double const PID::getPid(double error)
    {
        //Setting data variables
        this->error = error;
        this->derivative = this->error - this->prevError;
        this->integral += this->error;

        //Anti windup
            //Windup Range
            if(fabs(this->error) > fabs(this->windupRange))
                this->integral = 0;
            //Clamping
            if(fabs(this->integral) > fabs(this->maxIntegral))
                this->integral = sign(this->integral) * maxIntegral;

            //Sign-Flip Reset
            if(sign(this->prevError) != sign(this->error))
                this->integral = 0;
        
        this->prevError = this->error;
        return (Kp * this->error) + (Ki * this->integral) + (Kd * derivative);

    }
    double const PID::getPid(double currentPosition, double targetPosition)
    {
        return getPid(targetPosition - currentPosition);
    }

    //Getters
    double const PID::getError() {return this->error;}
    double const PID::getIntegral() {return this->integral;}
    double const PID::getDervative() {return this->derivative;}

    //Reset
    void PID::reset()
    {
        error = 0;
        integral = 0;
        derivative = 0;
        prevError = 0;
    }

}
