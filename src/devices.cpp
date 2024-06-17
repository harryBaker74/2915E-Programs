#include "../include/harryLibHeader/globals.h"
#include "../include/main.h"

//Misc
    //Controller
        pros::Controller Controller (CONTROLLER_MASTER);

    //IMU
        pros::IMU IMU(INERTIAL);
//ADI Ports

//Motors

    //Drivetrain
        pros::Motor leftFront(LEFT_MOTOR_FRONT);
        pros::Motor leftMid(LEFT_MOTOR_MID);
        pros::Motor leftBack(LEFT_MOTOR_BACK);

        pros::Motor rightFront(RIGHT_MOTOR_FRONT);
        pros::Motor rightMid(RIGHT_MOTOR_MID);
        pros::Motor rightBack(RIGHT_MOTOR_BACK);
    
    //Intake
        pros::Motor intake(INTAKE);




