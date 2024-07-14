#include "../include/main.h"
#include "../include/harryLibHeader/pid.hpp"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/velocityController.hpp"

//Wrapper for a PID with the right constants to convert desired motor RPM into Voltage.
namespace vController
{
        vController::vController(bool Ks)
        {
            this->Kv = 20;
            this->Ka = 10;
            this->Ks = 1000 * Ks;
            //Reason for toggleable Ks is that ks pretty much makes the drive have zero friction, which means that there is
            //a lot of oscillation when tuning some controllers like 1d pids, however its still useful for other controllers
        };

        double vController::rpmVelToVoltage(double a, double b)
        {
            int i =0;
            return i;
        }

        double vController::rpmVelToVoltage(double currentVelocity, double prevVelocity, double targetVelocity)
        {
            double derivative = currentVelocity - prevVelocity;

            double output = Kv * targetVelocity + this->Ka * derivative;
            output += sign(output) * Ks;

            return output;
        }

        void vController::startTuner(double velocity)
        {

            pros::Task task {[=, this]{

                runTuner = true;

                //Initializing left drivetrain motors
                pros::Motor leftMotor1(LEFT_MOTOR_FRONT, pros::MotorGears::blue);
                pros::Motor leftMotor2(LEFT_MOTOR_MID, pros::MotorGears::blue);
                pros::Motor leftMotor3(LEFT_MOTOR_BACK, pros::MotorGears::blue);
                pros::MotorGroup leftDrive(leftMotor1);
                leftDrive.append(leftMotor2);
                leftDrive.append(leftMotor3);

                //Counter to plot values on graph
                int counter = 0;

                double prevAvergeVelocity = 0;

                double Kv = 20;
                double Ka = 10;
                double Ks = 1000;

                //Controller Loop
                while(runTuner)
                {
                    //Getting current Velocities
                    std::vector <double> currentVelocities = leftDrive.get_actual_velocity_all();
                    double averageVelocity = (currentVelocities.at(0) + currentVelocities.at(1) + currentVelocities.at(2)) / 3;
                    double change = averageVelocity - prevAvergeVelocity;
                    prevAvergeVelocity = averageVelocity;

                    double output = Kv * velocity + this->Ka * change;
                    output += sign(output) * Ks;

                    //Getting Pid Ouput and changing plant based on that
                    leftDrive.move_voltage(output);

                    //Getting current error and printing it
                    double error = velocity - averageVelocity;
                    printf("(%d, %f)\n", counter, error);

                    counter++;

                    //Delay so we can stop this task in the main task
                    pros::delay(10);
                }

                //Killing task at the end
                pros::Task::current().remove();
            }}; 
        }

        void vController::endTuner()
        {
            runTuner = false;
        }
}