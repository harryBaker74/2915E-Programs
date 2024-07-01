#include "../include/main.h"
#include "../include/harryLibHeader/globals.h"
#include <vector>

namespace Odometery
{
    std::vector <std::vector<double>> currentEncoderValues;
    std::vector <std::vector<double>> prevEncoderValues;

    //Getters

    double getEncoder(int portConstant = LEFT_MOTOR_FRONT)
    {
        //Switch becuase it looks a bit better
        switch (portConstant)
        {
            case LEFT_MOTOR_FRONT: return currentEncoderValues.at(0).at(0);
            case LEFT_MOTOR_MID: return currentEncoderValues.at(0).at(1);
            case LEFT_MOTOR_BACK: return currentEncoderValues.at(0).at(2);

            case RIGHT_MOTOR_FRONT: return currentEncoderValues.at(1).at(0);
            case RIGHT_MOTOR_MID: return currentEncoderValues.at(1).at(1);
            case RIGHT_MOTOR_BACK: return currentEncoderValues.at(1).at(2);

            case TRACKING_WHEEL: return currentEncoderValues.at(2).at(0);

            case INERTIAL: return currentEncoderValues.at(3).at(0);

            default: return 0;
        }
    }
    //Specific getters
    std::vector<double> getLeftDriveEncoders()
    {return currentEncoderValues.at(0);}
    std::vector<double> getRightDriveEncoders()
    {return currentEncoderValues.at(1);}
    std::vector<double> getTrackingWheelEncoders()
    {return currentEncoderValues.at(2);}
    double getInertialRotation()
    {return currentEncoderValues.at(3).at(0);}
    
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
        double horizontalArcLength = (deltaHorizontal / (deltaHeading + 0.00000001)) + HORIZONTAL_OFFSET; //Preventing divide by zero if deltaHeading is zero

        //Calculating the arcs chord lengths
        //Can imagine these as being rotated by theta / 2, with the vertical arc's chord becoming the new y axis, and horizontal arc's chord becoming the new x axis.
        double localY = 2 * sin(deltaHeading / 2) * verticalArcLength;
        double localX = 2 * sin(deltaHeading / 2) * horizontalArcLength;

        //Rotataing the local axis back to global
        double averageHeading = robotPose->rotation + (deltaHeading / 2); //Amount to rotate by

        robotPose->x += localX * cos(averageHeading) - localY * sin(averageHeading);    //Applying rotation matrix
        robotPose->y += localX * sin(averageHeading) + localY * cos(averageHeading);    //Applying rotation matrix
        robotPose->rotation = currentEncoderValues.at(3).at(0);                         //Pure rotation amount no bounding, in rad
        robotPose->heading = boundAngle(robotPose->heading + deltaHeading, true);       //Heading bounded between -pi and pi, in rad

    }
}
