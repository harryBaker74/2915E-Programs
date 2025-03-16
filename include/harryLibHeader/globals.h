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

    //Track width
    #define TRACKWIDTH 10.5

    //Controller port
    #define CONTROLLER 8

    //MCL Distance Sensor ports
    #define DIST_LEFT 30
    #define DIST_FRONT 31
    #define DIST_RIGHT 32

    //IMU Port
    #define INERTIAL 14

    //Tracking Wheel Port
    #define TRACKING_WHEEL 3

    //Drivtrain Motor Ports
    #define LEFT_MOTOR_FRONT 1
    #define LEFT_MOTOR_MID 2
    #define LEFT_MOTOR_BACK -9

    #define RIGHT_MOTOR_FRONT -4
    #define RIGHT_MOTOR_MID -7
    #define RIGHT_MOTOR_BACK 8

//Other Systems
    //Intake Port
    #define INTAKE 3

    //Intake optical
    #define OPTICAL 6

    //Lady Brown Ports
    #define LIFT -10

    //Lady Brown Rotation
    #define ROTATION 11

//Solanoids
    //Mogo Port and controlss
    #define MOGO 'A'
    #define MOGO_CONTROL DIGITAL_L2

    //Doinker Port and controls
    #define DOINKER 'C'
    #define DOINKER_CONTROL DIGITAL_B

    //Rush Mech Port and controls
    #define RUSH 'B'
    #define RUSH_CONTROL DIGITAL_DOWN

    //PTO Port and controls
    #define PTO 'D'
    #define PTO_CONTROL DIGITAL_Y
