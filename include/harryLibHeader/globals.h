#pragma once

//Drivetrain things
    //Wheel Size
    #define DRIVE_WHEEL_DIAMETER 2.75
    #define TRACKING_WHEEL_DIAMETER 2.75
    #define DRIVE_GEAR_RATIO 1
    #define TRACKING_GEAR_RATIO 1

    //Tracking wheel offsets(cm)
    #define HORIZONTAL_OFFSET 2.61
    #define VERTICAL_OFFSET 0//2.60523824550331

    //Controller port
    #define CONTROLLER 8

    //IMU Port
    #define INERTIAL 11

    //Tracking Wheel Port
    #define TRACKING_WHEEL 3

    //Drivtrain Motor Ports
    #define LEFT_MOTOR_FRONT 9
    #define LEFT_MOTOR_MID -2
    #define LEFT_MOTOR_BACK -7

    #define RIGHT_MOTOR_FRONT -5
    #define RIGHT_MOTOR_MID 6
    #define RIGHT_MOTOR_BACK 8

//Other Systems
    //Intake Port
    #define INTAKE -4

    //Intake optical
    #define OPTICAL 20

    //Lady Brown Ports
    #define LIFT_1 -21
    #define LIFT_2 16

//Solanoids
    //Mogo Port and controlss
    #define MOGO 'B'
    #define MOGO_CONTROL DIGITAL_L2

    //Doinker Port and controls
    #define DOINKER 'A'
    #define DOINKER_CONTROL DIGITAL_X

    //Rush Mech Port and controls
    #define RUSH 'C'
    #define RUSH_CONTROL DIGITAL_B

    //PTO Port and controls
    #define PTO 'D'
    #define PTO_CONTROL DIGITAL_Y
