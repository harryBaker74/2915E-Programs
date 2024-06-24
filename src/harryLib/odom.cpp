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

    std::vector<double> getLeftDriveEncoders()
    {return currentEncoderValues.at(0);}
    std::vector<double> getRightDriveEncoders()
    {return currentEncoderValues.at(1);}
    std::vector<double> getTrackingWheelEncoders()
    {return currentEncoderValues.at(2);}
    
    void OdometeryCalculations(Pose* robotPose, pros::MotorGroup* leftDrive, pros::MotorGroup* rightDrive, pros::Rotation* trackingWheel)
    {

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
            }
        };


    }
}
