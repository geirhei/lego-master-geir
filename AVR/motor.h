/************************************************************************/
// File:			motor.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Register ticks from optical encoders mounted to
//                  cogwheels
//
// Note: Motor is active low
// ISR routines in main.c
/************************************************************************/


#ifndef MOTOR_H_
#define MOTOR_H_

/************************************************************************/
// Initialize the motor encoder and control pins
// Trigger on rising edge
// Input: pins PD2 and PD3 (INT0 and INT1)
// Note: ISR Routines are in main.c
/************************************************************************/
void vMotor_init();

/* Set up all interrupts */
void vMotor_interruptInit();

/************************************************************************/
// Control robot movement according to truth table - NB: Active low.
// Movement    | LeftBwrd      | LeftFwrd      | RightFwrd  | RightBwrd
// Forward     | HIGH          | LOW           | LOW        | HIGH
// Backward    | LOW           | HIGH          | HIGH       | LOW
// Rotate CCW  | LOW           | HIGH          | LOW        | HIGH
// Rotate CW   | HIGH          | LOW           | HIGH       | LOW
// Stop moving | HIGH          | HIGH          | HIGH       | HIGH
/************************************************************************/
void vMotorMoveLeftForward();
void vMotorMoveRightForward();

void vMotorMoveLeftBackward();
void vMotorMoveRightBackward();

void vMotorBrakeLeft();
void vMotorBrakeRight();

/* Soft PWM for motor logic. Actuation is set in percent "on" during    */
/* the duty cycle of 20 ms 0%(off) < 40%(min) < 100%(max)               */
/* NB1: This function HAS to be called from a task                      */
/* NB2: vTaskDelay(0) will cause this task to yield                     */
/* Movement can be moveForward, moveBackward,                           */
/* ... moveClockWise or moveCounterClockwise                            */
void vMotorMovementSwitch(uint8_t movement, int16_t tmp_leftWheelTicks, int16_t tmp_rightWheelTicks);

/* Handle ISR ticks from encoder */
void vMotorEncoderLeftTickFromISR(uint8_t wheelDirection);

/* Handle ISR ticks from encoder */
void vMotorEncoderRightTickFromISR(uint8_t wheelDirection);

// Return 1 if direction is forward, return 2 if it is backward, return 0 if it is not moving
uint8_t vMotorGetLeftDir(void);

// Return 1 if direction is forward, return 2 if it is backward, return 0 if it is not moving
uint8_t vMotorGetRightDir(void);

#endif /* MOTORENCODER_H_ */