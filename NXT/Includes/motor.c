#include "motor.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"

#include <math.h>

#include "defines.h"
#include "nxt_motors.h"
#include "display.h"


#define SPEED 50
#define TOWER_SPEED 20

#define TICKS_PER_DEGREE 7.1

extern volatile uint8_t gHandshook;

extern QueueHandle_t globalWheelTicksQ;

void vMotor_init(void) {
  nxt_motor_set_speed(servoLeft, 0, 1);
  nxt_motor_set_speed(servoRight, 0, 1);
  nxt_motor_set_speed(servoTower, 0, 1);
  
  nxt_motor_set_count(servoLeft, 0);
  nxt_motor_set_count(servoRight, 0);
  nxt_motor_set_count(servoTower, 0);
}

static void vMotorMoveLeftForward(uint8_t actuation, uint8_t *leftWheelDirection) {
    nxt_motor_set_speed(servoLeft, actuation, 1);
    *leftWheelDirection = motorForward;
}

static void vMotorMoveRightForward(uint8_t actuation, uint8_t *rightWheelDirection) {
	nxt_motor_set_speed(servoRight, actuation, 1);
	*rightWheelDirection = motorForward;
}

static void vMotorMoveLeftBackward(uint8_t actuation, uint8_t *leftWheelDirection) {
	nxt_motor_set_speed(servoLeft, -actuation, 1);
    *leftWheelDirection = motorBackward;
}

static void vMotorMoveRightBackward(uint8_t actuation, uint8_t *rightWheelDirection) {
	nxt_motor_set_speed(servoRight, -actuation, 1);
    *rightWheelDirection = motorBackward;
}

static void vMotorBrakeLeft() {
	nxt_motor_set_speed(servoLeft, 0, 1); // use break here?
}

static void vMotorBrakeRight() {
	nxt_motor_set_speed(servoRight, 0, 1); // use break here?
}

/* Switch for robot movement to abstract the logic away from main */
void vMotorMovementSwitch(int16_t leftSpeed, int16_t rightSpeed, uint8_t *leftWheelDirection, uint8_t *rightWheelDirection){
    if (leftSpeed > 0) {
		vMotorMoveLeftForward(leftSpeed, leftWheelDirection);
    } else if(leftSpeed < 0) {
		vMotorMoveLeftBackward(-leftSpeed, leftWheelDirection);
    } else {
		vMotorBrakeLeft();
	}
	
	if (rightSpeed > 0) {
		vMotorMoveRightForward(rightSpeed, rightWheelDirection);
	} else if (rightSpeed < 0) {
		vMotorMoveRightBackward(-rightSpeed, rightWheelDirection);
	} else {
		vMotorBrakeRight();
	}
}

void vMotorSetAngle(uint8_t motor, int16_t angle) {
  angle = -angle; //Application code uses opposite encoder values to the nxt
  if(angle < -90) angle = -90;
  nxt_motor_command(motor, (int) floor(angle*TICKS_PER_DEGREE), TOWER_SPEED);
}

void vMotorCountUpdate(void) {
  wheel_ticks_t WheelTicks;
  WheelTicks.right = nxt_motor_get_count(servoRight);
  WheelTicks.left = nxt_motor_get_count(servoLeft);
  // Pass the motor count values to the global queue containing wheel ticks
  xQueueOverwrite(globalWheelTicksQ, &WheelTicks);
}

/* Handle ISR ticks from encoder, Please note that we are losing accuracy here due to division */
void vMotorEncoderLeftTickFromISR(uint8_t wheelDirection, int16_t *leftWheelTicks, uint8_t leftEncoderTicks){
    switch (wheelDirection){
        case motorForward:{
            *leftWheelTicks += leftEncoderTicks / 2;
            break;
        }
        case  motorBackward:{
            *leftWheelTicks -= leftEncoderTicks / 2;
            break;
        }
        default:
        // We have a count when the robot is supposedly not moving.
        break;
    }
}
void vMotorEncoderRightTickFromISR(uint8_t wheelDirection, int16_t *rightWheelTicks, uint8_t rightEncoderTicks){
    switch (wheelDirection){
        case motorForward:{
            *rightWheelTicks += rightEncoderTicks / 2;
            break;
        }
        case  motorBackward:{
            *rightWheelTicks -= rightEncoderTicks / 2;
            break;
        }
        default:
        // We have a count when the robot is supposedly not moving.
        break;
    }
}