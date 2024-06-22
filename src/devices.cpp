#include "../include/harryLibHeader/globals.h"
#include "../include/main.h"

//Misc
    //Controller
        pros::Controller Controller (CONTROLLER_MASTER);

    //IMU
        pros::IMU IMU(INERTIAL);
//ADI Ports
    //Pneumatics
        //Mogo
            pros::adi::Pneumatics mogoSolanoid(MOGO, false);

//Motors

    //Drivetrain
        pros::Motor leftFrontMotor(LEFT_MOTOR_FRONT);
        pros::Motor leftMidMotor(LEFT_MOTOR_MID);
        pros::Motor leftBackMotor(LEFT_MOTOR_BACK);

        pros::Motor rightFrontMotor(RIGHT_MOTOR_FRONT);
        pros::Motor rightMidMotor(RIGHT_MOTOR_MID);
        pros::Motor rightBackMotor(RIGHT_MOTOR_BACK);
    
    //Intake
        pros::Motor intakeMotor(INTAKE);

    //Plunger
        pros::Motor plungerMotor(PLUNGER);




