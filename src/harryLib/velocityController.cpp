#include "../include/main.h"
#include "../include/harryLibHeader/pid.hpp"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/velocityController.hpp"

//Wrapper for a PID with the right constants to convert desired motor RPM into Voltage.
namespace vController
{
        vController::vController(){};

        double vController::rpmVelToVoltage(double currentVelocity, double targetVelocity)
        {
            return rpmPid.getPid(currentVelocity, targetVelocity);
        }

        void vController::reset()
        {
            rpmPid.reset();
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

                //Counter so we can plot values on graph
                int counter = 0;

                //Controller Loop
                while(runTuner)
                {
                    //Getting current Velocities
                    std::vector <double> currentVelocities = leftDrive.get_actual_velocity_all();
                    double averageVelocity = (currentVelocities.at(0) + currentVelocities.at(1) + currentVelocities.at(2)) / 3;

                    //Getting Pid Ouput and changing plant based on that
                    double output = rpmPid.getPid(averageVelocity, velocity);
                    leftDrive.move_voltage(output);

                    //Getting current error and printing it
                    double error = rpmPid.getError();
                    printf("(%d, %d)", counter, error);

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