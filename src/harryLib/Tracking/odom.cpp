#include "../include/main.h"
#include "../include/harryLibHeader/globals.h"
#include <vector>

namespace Odometery
{
    //Values for the robot model
    


    std::vector <std::vector<double>> currentEncoderValues = {{0,0,0},{0,0,0},{0},{0}};
    std::vector <std::vector<double>> prevEncoderValues = {{0,0,0},{0,0,0},{0},{0}};

    //Getters

    double getEncoder(int portConstant)
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

    double getLeftEncoder()
    {return currentEncoderValues.at(0).at(0);}
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

        //Updating model values

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
                IMU->get_rotation() * M_PI / 180
            }
        };

        //Getting the change in encoder values compared to lass time step
        double deltaLeft = (
            (currentEncoderValues.at(0).at(0) - prevEncoderValues.at(0).at(0)) +    //Delta LEFT_MOTOR_FRONT
            (currentEncoderValues.at(0).at(1) - prevEncoderValues.at(0).at(1)) +    //Delta LEFT_MOTOR_MID
            (currentEncoderValues.at(0).at(2) - prevEncoderValues.at(0).at(2))      //Delta LEFT_MOTOR_BACK
        ) / 3;
        
        double deltaRight = (
            (currentEncoderValues.at(1).at(0) - prevEncoderValues.at(1).at(0)) +    //Delta RIGHT_MOTOR_FRONT
            (currentEncoderValues.at(1).at(1) - prevEncoderValues.at(1).at(1)) +    //Delta RIGHT_MOTOR_MID
            (currentEncoderValues.at(1).at(2) - prevEncoderValues.at(1).at(2))      //Delta RIGHT_MOTOR_BACK
        ) / 3;

        double deltaHorizontal = (
        prevEncoderValues.at(2).at(0) - currentEncoderValues.at(2).at(0));       //Delta TRACKING_WHEEL

        double deltaHeading = (
            currentEncoderValues.at(3).at(0) - prevEncoderValues.at(3).at(0)         //Delta IMU Z-Axis
        );

        //Setting the previous encoder values to the current one for next time step
        prevEncoderValues = currentEncoderValues;

        //Converting Delta Encoders into cm moved
        deltaLeft = ((deltaLeft * M_PI / 180) * DRIVE_GEAR_RATIO) * inToCm(DRIVE_WHEEL_DIAMETER) / 2;
        deltaRight = ((deltaRight * M_PI / 180) * DRIVE_GEAR_RATIO) * inToCm(DRIVE_WHEEL_DIAMETER) / 2;
        deltaHorizontal = ((deltaHorizontal * M_PI / 180) * TRACKING_GEAR_RATIO) * inToCm(TRACKING_WHEEL_DIAMETER) / 2;

        //Odom Calculations

        //Calculating radius length for the vertical arc and horiontal arc
        //Preventing divide by zero if deltaHeading is zero
        deltaHeading += (deltaHeading == 0) * 0.000000001;
        double verticalArcRadius = std::max(((deltaLeft / deltaHeading) - VERTICAL_OFFSET), ((deltaRight / deltaHeading) + VERTICAL_OFFSET));
        double horizontalArcRadius = (deltaHorizontal / (deltaHeading) + HORIZONTAL_OFFSET); 


        //Calculating the arcs chord lengths
        //Can imagine these as being rotated by theta / 2, 
        //with the vertical arc's chord becoming the new y axis, and horizontal arc's chord becoming the new x axis.
        double localY = 2 * sin(deltaHeading / 2) * verticalArcRadius;
        double localX = 2 * sin(deltaHeading / 2) * horizontalArcRadius;

        //Rotataing the local axis back to global
        double averageHeading = robotPose->rotation + deltaHeading / 2; //Amount to rotate by

        //This rotation matrix might still be wrong i will see on friday
        robotPose->x += localX * cos(averageHeading) + localY * sin(averageHeading);   //Applying rotation matrix
        robotPose->y += (-localX * sin(averageHeading)) + localY * cos(averageHeading);    //Applying rotation matrix
        robotPose->rotation = currentEncoderValues.at(3).at(0);                         //Pure rotation amount no bounding, in rad
        robotPose->heading = boundAngle(robotPose->heading + deltaHeading, true);       //Heading bounded between -pi and pi, in rad
    }
}
