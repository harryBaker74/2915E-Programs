/*
pros::MotorGroup left_mg({10, -18, -19});    
pros::MotorGroup right_mg({8, 9, -20}); 

pros::Motor intakeT(-12);
pros::Motor intakeB(11);
pros::ADIDigitalOut mogo('g');
*/
#pragma once

//Wheel Size
#define DRIVE_WHEEL_DIAMETER 2.75
#define TRACKING_WHEEL_DIAMETER 2.75
#define DRIVE_GEAR_RATIO 0.8
#define TRACKING_GEAR_RATIO 1

//Tracking wheel offsets(cm)
#define HORIZONTAL_OFFSET 0
#define VERTICAL_OFFSET 13

//Controller port
#define CONTROLLER 1

//IMU Port
#define INERTIAL 17

//Tracking Wheel Port
#define TRACKING_WHEEL 3

//Mogo Port
#define MOGO 'G'

//Drivtrain Motor Ports
#define LEFT_MOTOR_FRONT 7
#define LEFT_MOTOR_MID -18
#define LEFT_MOTOR_BACK -19

#define RIGHT_MOTOR_FRONT 8
#define RIGHT_MOTOR_MID 9
#define RIGHT_MOTOR_BACK -20

//Intake Port
#define INTAKE_T 12
#define INTAKE_B 11

//Basket Port
#define BASKET_L 13
#define BASKET_R -14
