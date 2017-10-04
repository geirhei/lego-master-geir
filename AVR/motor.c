/************************************************************************/
// File:			motor.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Register ticks from optical encoders mounted to
//                  cogwheels
//
// Note: Motor is active low
// ISR routines in main.c
/************************************************************************/

/* AVR INCLUDES    */
#include <avr/io.h>
#include <avr/interrupt.h>

/*  Custom includes    */
#include "defines.h"
#include "motor.h"
#include "LED.h"

/* Function vMotorEncoderTickFromISR() uses global variables */
extern volatile int16_t gLeftWheelTicks;
extern volatile int16_t gRightWheelTicks;
/* Functions vMotorMove...() uses global variables */
extern uint8_t gLeftWheelDirection;
extern uint8_t gRightWheelDirection;

/************************************************************************/
// Initialize the motor encoder and control pins
// Initialize interrupt for nRF51 dongle
// Trigger on rising edge
// Input: pins PD2 and PD3 (INT0 and INT1)
// Note: ISR Routines are in main.c
/************************************************************************/
void vMotor_init(){
    /* Initialize motor pins as output */
    motorRegRight |= (1<<motorRightBackward) | (1<<motorRightForward);
    motorRegLeft |= (1<<motorLeftForward) | (1<<motorLeftBackward);
    
    /* Set all motor pins high to ensure zero movement */
    motorPortRight |= (1<<motorRightBackward) | (1<<motorRightForward);
    motorPortLeft |= (1<<motorLeftForward) | (1<<motorLeftBackward);
}
/* Set up all interrupts */
void vMotor_interruptInit(){
    /* Initialize motor encoder pins as input */
    encoderReg &= ~((1<<encoderPinRight) & (1<<encoderPinLeft));
    
    /* Clear interrupt enable bits to ensure no interrupts occur */
    EIMSK &= ~( (1<<INT0) & (1<<INT1) & (1<<INT2) );
    
    /* Set interrupt to trigger on rising edge for motor and all for nRF51 dongle */
    /* Datasheet p66 table 11-1 */
    EICRA |= (1<<ISC01) | (1<<ISC00) | (1<<ISC11) | (1<<ISC10) |  (0<<ISC21) | (1<<ISC20);
    
    /* Clear interrupt flag for INT0, INT1 and INT2*/
    EIFR = (1<<INTF0) | (1<<INTF1) |  (1<<INT2);
    
    /* Enable interrupts for INT0, INT1 and INT2*/
    EIMSK |= (1<<INT0) | (1<<INT1) | (1<<INT2);    
}

/************************************************************************/
// Control robot movement according to truth table - NB: Active low.
// Movement    | LeftBwrd      | LeftFwrd      | RightFwrd  | RightBwrd
// --------------------------------------------------------------------
// Forward     | HIGH          | LOW           | LOW        | HIGH
// Backward    | LOW           | HIGH          | HIGH       | LOW
// Rotate CCW  | LOW           | HIGH          | LOW        | HIGH
// Rotate CW   | HIGH          | LOW           | HIGH       | LOW
// Stop moving | HIGH          | HIGH          | HIGH       | HIGH
/************************************************************************/
void vMotorMoveLeftForward(){
    motorPortLeft |= (1<<motorLeftBackward);
    motorPortLeft &= ~(1<<motorLeftForward);
    gLeftWheelDirection = motorLeftForward;
}
void vMotorMoveRightForward(){
    motorPortRight |= (1<<motorRightBackward);
    motorPortRight &= ~(1<<motorRightForward);
    gRightWheelDirection = motorRightForward;
}

void vMotorMoveLeftBackward(){
    motorPortLeft &= ~(1<<motorLeftBackward);
    motorPortLeft |= (1<<motorLeftForward);
    gLeftWheelDirection = motorLeftBackward;
}
void vMotorMoveRightBackward(){
    motorPortRight &= ~(1<<motorRightBackward);
    motorPortRight |= (1<<motorRightForward);
    gRightWheelDirection = motorRightBackward;
}

void vMotorBrakeLeft(){
    motorPortLeft |= (1<<motorLeftForward) | (1<<motorLeftBackward);
    /* Do not set any direction to motor here, it can overshoot */
}

void vMotorBrakeRight(){
    motorPortRight |= (1<<motorRightForward) | (1<<motorRightBackward);
    /* Do not set any direction to motor here, it can overshoot */
};

/* Switch for robot movement to abstract the logic away from main */
/* Comparing ticks ensures balanced movement */
void vMotorMovementSwitch(uint8_t movement, int16_t tmp_leftWheelTicks, int16_t tmp_rightWheelTicks){
    switch (movement){
        case moveForward:{
            if (tmp_leftWheelTicks > tmp_rightWheelTicks){
                vMotorBrakeLeft();
                vMotorMoveRightForward();
            }
            else if (tmp_rightWheelTicks > tmp_leftWheelTicks){
                vMotorBrakeRight();
                vMotorMoveLeftForward();
            }
            else{
                vMotorMoveLeftForward();
                vMotorMoveRightForward();
            }
            break;
        }
        case moveBackward:{
            if (tmp_leftWheelTicks < tmp_rightWheelTicks){
                vMotorBrakeLeft();
                vMotorMoveRightBackward();
            }
            else if (tmp_rightWheelTicks < tmp_leftWheelTicks){
                vMotorBrakeRight();
                vMotorMoveLeftBackward();
            }
            else{
                vMotorMoveLeftBackward();
                vMotorMoveRightBackward();
            }
            break;
        }
        case moveClockwise:{
            if(-tmp_rightWheelTicks > tmp_leftWheelTicks){
                vMotorBrakeRight();
                vMotorMoveLeftForward();
                }else if (tmp_leftWheelTicks > -tmp_rightWheelTicks){
                vMotorBrakeLeft();
                vMotorMoveRightBackward();
            }
            else{
                vMotorMoveLeftForward();
                vMotorMoveRightBackward();
            }
            break;
        }
        case moveRightForward:{
            vMotorMoveRightForward();
            vMotorBrakeLeft();
            break;
        }
        case moveLeftForward:{
            vMotorMoveLeftForward();
            vMotorBrakeRight();
            break;
        }
        case moveRightBackward:{
            vMotorMoveRightBackward();
            vMotorBrakeLeft();
            break;
        }
        case moveLeftBackward:{
            vMotorMoveLeftBackward();
            vMotorBrakeRight();
            break;
        }
        case moveCounterClockwise:{
            if(-tmp_leftWheelTicks > tmp_rightWheelTicks){
                vMotorBrakeLeft();
                vMotorMoveRightForward();
            }
            else if (tmp_rightWheelTicks > -tmp_leftWheelTicks){
                vMotorBrakeRight();
                vMotorMoveLeftBackward();
            }
            else {
                vMotorMoveLeftBackward();
                vMotorMoveRightForward();
            }
            break;
        }
        default:{
            vMotorBrakeLeft();
            vMotorBrakeRight();
            break;
        }
        
    }

}

/* Handle ISR ticks from encoder */
void vMotorEncoderLeftTickFromISR(uint8_t wheelDirection){
    switch (wheelDirection){
        case motorLeftForward:{
            gLeftWheelTicks++;
            break;
        }
        case  motorLeftBackward:{
            gLeftWheelTicks--;
            break;
        }
        default:
        // We have a count when the robot is supposedly not moving.
        break;
    }
}
void vMotorEncoderRightTickFromISR(uint8_t wheelDirection){
    switch (wheelDirection){
        case motorRightForward:{
            gRightWheelTicks++;
            break;
        }
        case  motorRightBackward:{
            gRightWheelTicks--;
            break;
        }
        default:
        // We have a count when the robot is supposedly not moving.
        break;
    }
}

// Return 1 if direction is forward, return 2 if it is backward, return 0 if it is not moving
uint8_t vMotorGetLeftDir(void){
        if (!(motorPortLeft & (1<<motorLeftForward)) && motorPortLeft & (1<<motorLeftBackward)){
            return 1;
        }
        else if (motorPortLeft & (1<<motorLeftForward) && !(motorPortLeft & (1<<motorLeftBackward))){
            return 2;
        }
        else return 0;
}

// Return 1 if direction is forward, return 2 if it is backward, return 0 if it is not moving
uint8_t vMotorGetRightDir(void){
    if (!(motorPortRight & (1<<motorRightForward)) && motorPortRight & (1<<motorRightBackward)){
        return 1;
    }
    else if (motorPortRight & (1<<motorRightForward) && !(motorPortRight & (1<<motorRightBackward))){
        return 2;
    }
    else return 0;
}

