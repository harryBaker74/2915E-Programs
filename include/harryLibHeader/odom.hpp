#pragma once
#include "main.h"
#include "globals.h"

namespace Odometery
{

    // struct Particle
    // {
    //     double x;
    //     double y;
    //     double weight;

    //     //Constructor
    //     Particle();
    //     Particle(double x, double y, double weight);
    // };

    // pros::Distance distanceLeft (DIST_LEFT);
    // pros::Distance distanceBack (DIST_BACK);
    // pros::Distance distanceRight (DIST_RIGHT);
    // std::vector<Particle> particles;

    extern std::vector<std::vector<double>> currentEncoderValues;
    extern std::vector<std::vector<double>> prevEncoderValues;

    
    
    //Getters
    /**
     * @brief Gets the currently tracked encoder value from the inputted port.
     * 
     * @param portConstant The constant of the port for the desired device
     */
    double getEncoder(int portConstant = LEFT_MOTOR_FRONT);
    double getLeftEncoder();
    //Specific getters
    //Returns a vector containg the left drivetrains encoder values in order
    std::vector<double> getLeftDriveEncoders();
    //Returns a vector containg the right drivetrains encoder values in order
    std::vector<double> getRightDriveEncoders();
    //Returns a vector containg the tracking wheels encoder values in order
    std::vector<double> getTrackingWheelEncoders();
    //Returns a double with the unbounded z-axis rotation of the inertial sensor
    double getInertialRotation();
    //Returns a vector with all the distance sensor readings for MCL (in cm). Left, then back, then right
    std::vector<double> getDistSensReadings();
    //Returns the value of the distance sensor from whatever port was put in (in cm)
    double getDistSensRead(int portConstant = DIST_LEFT);

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


    //MCL Functions

    /**
     * @brief Genrates a set of 2 dimensional particles distributed around the mean with a gaussian distribution.
     */
    void MCLGenerateParticles(Point meanPoint, double std, double particleAmount);
    
    /**
     * @brief Removes low weighted particles, and duplicates higher wieghted particles
     */
    void MCLResample();

    /**
     * @brief Moves mcl particles based on a mean delat position and a standard deviation from that mean
     */
    void MCLPredict(Point deltaPos, std::pair<double, double> std);

    /**
     * @brief Recalculates weights of every particle
     */
    void MCLUpdate(double std);

    /**
     * @brief Estimates where the robot is based off of the particles, and sets the global position of the robot to that
     */
    void MCLEstimate();
}
