#include "../include/main.h"
#include "../include/harryLibHeader/globals.h"
#include <vector>

namespace Odometery
{
    std::vector <std::vector<double>> currentEncoderValues;
    std::vector <std::vector<double>> prevEncoderValues;

    //Getters

    /**
     * @brief Gets the currently tracked encoder value from the inputted motor port.
     * 
     * @param motorPortConstant The constant of the port for the desired motor
     */
    double getEncoder(int motorPortConstant = LEFT_MOTOR_FRONT)
    {
        //Switch becuase it looks a bit better
        switch (motorPortConstant)
        {
            case LEFT_MOTOR_FRONT: return currentEncoderValues.at(0).at(0);
            case LEFT_MOTOR_MID: return currentEncoderValues.at(0).at(1);
            case LEFT_MOTOR_BACK: return currentEncoderValues.at(0).at(2);

            case RIGHT_MOTOR_FRONT: return currentEncoderValues.at(1).at(0);
            case RIGHT_MOTOR_MID: return currentEncoderValues.at(1).at(1);
            case RIGHT_MOTOR_BACK: return currentEncoderValues.at(1).at(2);

            case TRACKING_WHEEL: return currentEncoderValues.at(2).at(0);
        }
    }
    //
    std::vector<double> getLeftDriveEncoders()
    {return currentEncoderValues.at(0);}
    std::vector<double> getRightDriveEncoders()
    {return currentEncoderValues.at(1);}
    std::vector<double> getTrackingWheelEncoders()
    {return currentEncoderValues.at(2);}
    

    /**
     * @brief Function to calculate new robot pose with inputed pose + trackers.
     * 
     * @param robotPose Reference to the global pose of the robot
     * @param leftDrive Reference to the left drive motor group for the robot
     * @param rightDrive Reference to the right drive motor group for the robot
     * @param trackingWheel Reference to tracking wheel(s) rotation sensor
     */
    void OdometeryCalculations(Pose* robotPose, pros::MotorGroup* leftDrive, pros::MotorGroup* rightDrive, pros::Rotation* trackingWheel, pros::IMU* IMU)
    {

        //Getting current encoder values
        currentEncoderValues = 
        {   {
            leftDrive->get_position(0),
            leftDrive->get_position(1),
            leftDrive->get_position(2)
            }, 
            {
            rightDrive->get_position(0),
            rightDrive->get_position(1),
            rightDrive->get_position(2)
            }, 
            {
            double(trackingWheel->get_position() * CENTIDEGREES_TO_ENCODER)
            },
            {
                IMU->get_rotation()
            }
        };

        //Getting the change in encoder values compared to lass time step
        double deltaLeft = (
            (prevEncoderValues.at(0).at(0) - currentEncoderValues.at(0).at(0)) +    //Delta LEFT_MOTOR_FRONT
            (prevEncoderValues.at(0).at(1) - currentEncoderValues.at(0).at(1)) +    //Delta LEFT_MOTOR_MID
            (prevEncoderValues.at(0).at(2) - currentEncoderValues.at(0).at(2))      //Delta LEFT_MOTOR_BACK
        ) / 3;
        
        double deltaRight = (
            (prevEncoderValues.at(1).at(0) - currentEncoderValues.at(1).at(0)) +    //Delta RIGHT_MOTOR_FRONT
            (prevEncoderValues.at(1).at(1) - currentEncoderValues.at(1).at(1)) +    //Delta RIGHT_MOTOR_MID
            (prevEncoderValues.at(1).at(2) - currentEncoderValues.at(1).at(2))      //Delta RIGHT_MOTOR_BACK
        ) / 3;

        double deltaHorizontal = (
            prevEncoderValues.at(2).at(0) - currentEncoderValues.at(2).at(0)        //Delta TRACKING_WHEEL
        );

        double deltaHeading = (
            prevEncoderValues.at(3).at(0) - currentEncoderValues.at(3).at(0)        //Delta IMU Z-Axis
        );

        //Setting the previous encoder values to the current one for next time step
        prevEncoderValues = currentEncoderValues;

        //Calculating arc length for the vertical tracking wheel arc, and the horizontal tracking wheel arc
        double verticalArcLength = (deltaLeft + deltaRight)/ 2;
        double horizontalArcLength = (deltaHorizontal / deltaHeading) + HORIZONTAL_OFFSET;

        //Calculating the arcs chord lengths
        //Can imagine these as being rotated by theta / 2, with the vertical arc's chord becoming the new y axis, and horizontal arc's chord becoming the new x axis.
        double localY = 2 * sin(deltaHeading / 2) * verticalArcLength;
        double localX = 2 * sin(deltaHeading / 2) * horizontalArcLength;

        //Rotataing the local axis back to global



    }
}
