#pragma once

//Misc
    //Controller
        extern pros::Controller Controller;

    //IMU
        extern pros::IMU IMU;
//ADI Ports
    //Pistons
        //Mogo
            extern pros::adi::Pneumatics mogoSolanoid;

//Motors

    //Drivetrain
        extern pros::Motor leftFrontMotor;
        extern pros::Motor leftMidMotor;
        extern pros::Motor leftBackMotor;

        extern pros::Motor rightFrontMotor;
        extern pros::Motor rightMidMotor;
        extern pros::Motor rightBackMotor;
    
    //Intake
        extern pros::Motor intakeMotor;

    //Plunger
        extern pros::Motor plungerMotor;