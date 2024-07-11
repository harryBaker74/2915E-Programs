/*
pros::MotorGroup left_mg({10, -18, -19});    
pros::MotorGroup right_mg({8, 9, -20}); 

pros::Motor intakeT(-12);
pros::Motor intakeB(11);
pros::ADIDigitalOut mogo('g');
*/
#pragma once

//Tracking wheel offsets(cm)
#define HORIZONTAL_OFFSET 0
#define VERTICAL_OFFSET 0

//Controller port
#define CONTROLLER 1

//IMU Port
#define INERTIAL 17

//Tracking Wheel Port
#define TRACKING_WHEEL 3

//Mogo Port
#define MOGO 'G'

//Arm Piston Port
#define ARM_PISTON 'A'

//Clamp Piston Port
#define CLAMP_PISTON 'A'

//Drivtrain Motor Ports
#define LEFT_MOTOR_FRONT 10
#define LEFT_MOTOR_MID -18
#define LEFT_MOTOR_BACK -19

#define RIGHT_MOTOR_FRONT 8
#define RIGHT_MOTOR_MID 9
#define RIGHT_MOTOR_BACK -20

//Intake Port
#define INTAKE_T 12
#define INTAKE_B 11

//Plunger Port
#define PLUNGER 1
