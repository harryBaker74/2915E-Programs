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
    #define HORIZONTAL_OFFSET 17.145
    #define VERTICAL_OFFSET 0//2.60523824550331

    //Mcl offsets from tracking center(cm)
    #define MCL_LEFT_X_OFFSET -13.97
    #define MCL_LEFT_Y_OFFSET -1
    #define MCL_FRONT_X_OFFSET 3.175
    #define MCL_FRONT_Y_OFFSET 10.795
    #define MCL_RIGHT_X_OFFSET 13.97
    #define MCL_RIGHT_Y_OFFSET -1

    //Track width
    #define TRACKWIDTH 10.5

    //Controller port
    #define CONTROLLER 8

    //MCL Distance Sensor ports
    #define DIST_LEFT 6
    #define DIST_FRONT 15
    #define DIST_RIGHT 20

    //IMU Port
    #define INERTIAL 17

    //Tracking Wheel Port
    #define TRACKING_WHEEL 1

    //Drivtrain Motor Ports
    #define LEFT_MOTOR_FRONT 11
    #define LEFT_MOTOR_MID -12
    #define LEFT_MOTOR_BACK -13

    #define RIGHT_MOTOR_FRONT 18
    #define RIGHT_MOTOR_MID 19
    #define RIGHT_MOTOR_BACK -20

//Other Systems
    //Intake Port
    #define LOWER_INTAKE 16
    #define UPPER_INTAKE -15
//Solanoid systems
    #define MATCHLOAD 'h'
    #define HOOD 'g'
    #define DESCORE 'f'
