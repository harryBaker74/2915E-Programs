#pragma once

//Wall Positions
    #define FIELD_WIDTH 365.76
    #define FIELD_HEIGHT 365.76

//Drivetrain things
    //Wheel Size
    #define DRIVE_WHEEL_DIAMETER 3.25
    #define TRACKING_WHEEL_DIAMETER 3.25
    #define DRIVE_GEAR_RATIO 0.75
    #define TRACKING_GEAR_RATIO 1

    //Tracking wheel offsets(cm)
    #define HORIZONTAL_OFFSET 14.03//17.145
    #define VERTICAL_OFFSET 0//2.60523824550331

    //Mcl offsets from tracking center(cm)
    #define MCL_LEFT_OFFSET 12.1
    #define MCL_BACK_OFFSET 11.4
    #define MCL_RIGHT_OFFSET 12.1

    //Track width
    #define TRACKWIDTH 10.5

    //Controller port
    #define CONTROLLER 8

    //MCL Distance Sensor ports
    #define DIST_LEFT 15
    #define DIST_BACK 18
    #define DIST_RIGHT 12

    //IMU Port
    #define INERTIAL 2

    //Tracking Wheel Port
    #define TRACKING_WHEEL 3

    //Drivtrain Motor Ports
    #define LEFT_MOTOR_FRONT -16
    #define LEFT_MOTOR_MID -17
    #define LEFT_MOTOR_BACK -19

    #define RIGHT_MOTOR_FRONT 10
    #define RIGHT_MOTOR_MID 21
    #define RIGHT_MOTOR_BACK 20

//Other Systems
    //Intake Ports
    //This is  lower stage 11w motor
    #define LOWER_INTAKE_1 -14
    //This is lower stage 5.5w motor
    #define LOWER_INTAKE_2 -9

    //This is upper stage 5.5w
    #define UPPER_INTAKE 11

    //Optical for colour sort
    #define OPTICAL 1
    
    //Solanoid systems
    #define MATCHLOAD 'g'
    #define HOOD 'h'
    #define DESCORE 'f'