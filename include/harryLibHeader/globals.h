#pragma once

//Drivetrain things
    //Wheel Size
    #define DRIVE_WHEEL_DIAMETER 2.75
    #define TRACKING_WHEEL_DIAMETER 2.75
    #define DRIVE_GEAR_RATIO 0.75
    #define TRACKING_GEAR_RATIO 1

    //Tracking wheel offsets(cm)
    #define HORIZONTAL_OFFSET 2.61
    #define VERTICAL_OFFSET 14.5503391594702

    //Controller port
    #define CONTROLLER 8

    //IMU Port
    #define INERTIAL 10

    //Tracking Wheel Port
    #define TRACKING_WHEEL 9

    //Drivtrain Motor Ports
    #define LEFT_MOTOR_FRONT -2
    #define LEFT_MOTOR_MID -3
    #define LEFT_MOTOR_BACK 4

    #define RIGHT_MOTOR_FRONT 17
    #define RIGHT_MOTOR_MID -18
    #define RIGHT_MOTOR_BACK 19

//Other Motors
    //Intake Port
    #define INTAKE -20

    //Lady Brown Ports
    #define LIFT_1 0
    #define LIFT_2 0

//Solanoids
    //Mogo Port and controls
    #define MOGO 'A'
    #define MOGO_CONTROL DIGITAL_A

    //Doinker Port and controls
    #define DOINKER 'B'
    #define DOINKER_CONTROL DIGITAL_X

    //Rush Mech Port and controls
    #define RUSH 'C'
    #define RUSH_CONTROL DIGITAL_B

    //PTO Port and controls
    #define PTO 'D'
    #define PTO_CONTROL DIGITAL_Y
