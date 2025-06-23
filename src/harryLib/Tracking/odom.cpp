#include "../include/main.h"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/odom.hpp"
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

    double leftEncoders = 0;
    
    bool flipper = false;
    Point deltaPos(0, 0);

    Point prevPose = Point(0, 0);

    void OdometeryCalculations(Pose* robotPose, pros::MotorGroup* leftDrive, pros::MotorGroup* rightDrive, pros::Rotation* trackingWheel, pros::IMU* IMU)
    {
        prevPose = Point(*robotPose);
        //Updating mcl check
        flipper = !flipper;

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

        double deltaTracking = (
        prevEncoderValues.at(2).at(0) - currentEncoderValues.at(2).at(0)            //Delta TRACKING_WHEEL
        );       

        double deltaRotation = (
            currentEncoderValues.at(3).at(0) - prevEncoderValues.at(3).at(0)         //Delta IMU Z-Axis
        );

        //Setting the previous encoder values to the current one for next time step
        prevEncoderValues = currentEncoderValues;

        //Converting Delta Encoders into cm moved
        deltaLeft = ((deltaLeft * M_PI / 180) * DRIVE_GEAR_RATIO) * inToCm(DRIVE_WHEEL_DIAMETER) / 2;
        deltaRight = ((deltaRight * M_PI / 180) * DRIVE_GEAR_RATIO) * inToCm(DRIVE_WHEEL_DIAMETER) / 2;
        deltaTracking = ((deltaTracking * M_PI / 180) * TRACKING_GEAR_RATIO) * inToCm(TRACKING_WHEEL_DIAMETER) / 2;

        //Odom Calculations

        //Calculating the arcs chord lengths
        //Can imagine these as being rotated by theta / 2, a
        //with the vertical arc's chord becoming the new y axis, and horizontal arc's chord becoming the new x axis.
        double localY = 0;
        double localX = 0;
        if(deltaRotation == 0)
        {
            // localX = deltaHorizontal;
            localY = deltaLeft;        
        }
        else
        {
            // localX = 2 * sin(deltaRotation / 2) * (deltaHorizontal / deltaRotation - 4.27445897204689);
            localY = 2 * sin(deltaRotation / 2) * (deltaLeft / deltaRotation - 14.0995490417136);
        }


        //Rotataing the local axis back to global
        double averageHeading = robotPose->rotation + deltaRotation / 2; //Amount to rotate by

        //This rotation matrix might still be wrong i will see on friday
        robotPose->x += localX * cos(averageHeading) + localY * sin(averageHeading);   //Applying rotation matrix
        robotPose->y += (-localX * sin(averageHeading)) + localY * cos(averageHeading);    //Applying rotation matrix
        robotPose->rotation += deltaRotation;                         //Pure rotation amount no bounding, in rad
        robotPose->heading = boundAngle(robotPose->heading + deltaRotation, true);       //Heading bounded between -pi and pi, in rad

        deltaPos = deltaPos + (Point(*robotPose) - prevPose);

        Controller.print(0, 0, "%.2f, %.2f", robotPose->x, robotPose->y);

        //Offset calculator
        // leftEncoders += deltaHorizontal;
        // Controller.print(0, 0, "%.2f, %.2f", leftEncoders, robotPose->rotation);

        // If we should run mcl
        if(flipper)
        {
            // MCLPredict(deltaPos, std::make_pair(0.25, 0.25));
            // MCLUpdate(*robotPose, 0.25);
            // Point result = MCLEstimate();
            // robotPose->x = result.x;
            // robotPose->y = result.y;

            // deltaPos = Point(0, 0);
            // printf("%f, %f\n", localX, cos(averageHeading));
        }
        else
        {
            // MCLResample();
        }



        // Controller.print(0, 0, "%.1f, %.1f, %d", robotPose->x, robotPose->y, pros::millis());

    }
}
