#include "../include/main.h"
#include "../include/harryLibHeader/pid.hpp"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/velocityController.hpp"

//Velocity Controller with the right constants to convert desired motor RPM into Voltage.
namespace vController
{
        vController::vController(bool Ks)
        {
            this->Kv = 0.0;
            this->Ka = 0.0;
            this->Ks = 100.0 * Ks;
            //Reason for toggleable Ks is that ks pretty much makes the motor have zero friction, which means that there is
            //a lot of oscillation when tuning some controllers like 1d pids on drive, however its still useful for other controllers
        };

        double vController::rpmVelToVoltage(double currentVelocity, double targetVelocity)
        {
            double derivative = targetVelocity - currentVelocity;

            double output = (Kv * targetVelocity) + (Ka * derivative) + (sign(output) * Ks);
            return output;
        }

        void vController::tuner()
        {
            //Initializing left drivetrain motors
            pros::Motor leftMotor1(LEFT_MOTOR_FRONT, pros::MotorGears::blue);
            pros::Motor leftMotor2(LEFT_MOTOR_MID, pros::MotorGears::blue);
            pros::Motor leftMotor3(LEFT_MOTOR_BACK, pros::MotorGears::blue);
            pros::MotorGroup leftDrive(leftMotor1);
            leftDrive.append(leftMotor2);
            leftDrive.append(leftMotor3);

            double uStep = 6000;
            double targetVel = 300;
            double timer = 0;


            //Controller Loop
            while(true)
            {
                //Getting current Velocities
                std::vector <double> currentVelocities = leftDrive.get_actual_velocity_all();
                double averageVelocity = (currentVelocities.at(0) + currentVelocities.at(1) + currentVelocities.at(2)) / 3;
                
                double output = rpmVelToVoltage(averageVelocity, targetVel);

                //Getting Pid Ouput and changing plant based on that
                leftDrive.move_voltage(output);
                
                printf("(%f, %f)\n", timer, averageVelocity);
                
                timer += 1;
                //Delay so we can stop this task in the main task
                pros::delay(10);
            }
/*
            double inputVoltage = 12000;
            while(true)
            {
                
                leftDrive.move_voltage(inputVoltage);

                std::vector <double> currentVelocities = leftDrive.get_actual_velocity_all();
                double averageVelocity = (currentVelocities.at(0) + currentVelocities.at(1) + currentVelocities.at(2)) / 3;

                printf("(%f, %f)\n", inputVoltage, averageVelocity);
            }
*/
        }
}

//(1000.000000, 49.666667), (2000.000000, 105.133333), (3000.000000, 176.066667), (4000.000000, 230.133333), (5000.000000, 279.200000), (6000.000000, 341.800000), (7000.000000, 390.533333), (8000.000000, 446.866667), (9000.000000, 496.666667), (10000.000000, 538.533333), (11000.000000, 570.666667), (12000.000000, 605.266667)
//
//
//
//(
//
//
//
//
//
//
//

