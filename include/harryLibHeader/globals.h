#pragma once

//Wall Positions
    #define FIELD_WIDTH 365.76
    #define FIELD_HEIGHT 365.76

//Drivetrain things
    //Wheel Size
    #define DRIVE_WHEEL_DIAMETER 2.75
    #define TRACKING_WHEEL_DIAMETER 2.75
    #define DRIVE_GEAR_RATIO 1
    #define TRACKING_GEAR_RATIO 1

    //Tracking wheel offsets(cm)
    #define HORIZONTAL_OFFSET 2.61
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
    #define INERTIAL 5

    //Tracking Wheel Port
    #define TRACKING_WHEEL 10

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
    #define INTAKE_OPTICAL 21

    //Auto clamp
    #define MOGO_OPTICAL 19

    //Lady Brown Ports
    #define LIFT -11

    //Lift Optical
    #define LIFT_OPTICAL 18

    //Lady Brown Rotation
    #define ROTATION 16

//Solanoid systems
    //Mogo Port and controlss
    #define MOGO 'A'
    #define MOGO_CONTROL DIGITAL_L2

    //Left Doinker Port and controls
    #define LEFT_DOINKER 'C'
    #define LEFT_DOINKER_CONTROL DIGITAL_B

    //Right Doinker Port and controls
    #define RIGHT_DOINKER 'B'
    #define RIGHT_DOINKER_CONTROL DIGITAL_DOWN 

    //Intake Lift
    #define INTAKE_LIFT 'G'
    #define INTAKE_LIFT_CONTROL DIGITAL_UP


    //Goal Grabber
    #define GOAL_GRABBER 'H'
    #define GOAL_GRABBER_CONTROL DIGITAL_LEFT
