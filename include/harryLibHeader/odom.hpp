#pragma once
#include "main.h"
#include "globals.h"

namespace Odometery
{

    //Getters
    /**
     * @brief Gets the currently tracked encoder value from the inputted port.
     * 
     * @param portConstant The constant of the port for the desired device
     */
    double getEncoder(int portConstant = LEFT_MOTOR_FRONT);
    //Specific getters
    //Returns a vector containg the left drivetrains encoder values in order
    std::vector<double> getLeftDriveEncoders();
    //Returns a vector containg the right drivetrains encoder values in order
    std::vector<double> getRightDriveEncoders();
    //Returns a vector containg the tracking wheels encoder values in order
    std::vector<double> getTrackingWheelEncoders();
    //Returns a double with the unbounded z-axis rotation of the inertial sensor
    double getInertialRotation();

    /**
     * @brief Function to calculate new robot pose with inputed pose + trackers.
     * 
     * @param robotPose Reference to the global pose of the robot
     * @param leftDrive Reference to the left drive motor group for the robot
     * @param rightDrive Reference to the right drive motor group for the robot
     * @param trackingWheel Reference to tracking wheel(s) rotation sensor
     * @param IMU Rerference to the inertial sensor
     */
    void OdometeryCalculations(Pose* robotPose, pros::MotorGroup* leftDrive, pros::MotorGroup* rightDrive, pros::Rotation* trackingWheel, pros::IMU* IMU);
}
