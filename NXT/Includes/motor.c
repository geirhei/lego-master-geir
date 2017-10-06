#include "motor.h"
#include "defines.h"
#include "math.h"
#include "nxt_motors.h"
#include "display.h"

#define SPEED 50
#define TOWER_SPEED 20

#define TICKS_PER_DEGREE 7.1

extern int16_t gLeftWheelTicks;
extern int16_t gRightWheelTicks;

void vMotor_init(void) {
  nxt_motor_set_speed(servoLeft, 0, 1);
  nxt_motor_set_speed(servoRight, 0, 1);
  nxt_motor_set_speed(servoTower, 0, 1);
  
  nxt_motor_set_count(servoLeft, 0);
  nxt_motor_set_count(servoRight, 0);
  nxt_motor_set_count(servoTower, 0);
}
/* Switch for robot movement to abstract the logic away from main */
/* Comparing ticks ensures balanced movement */
void vMotorMovementSwitch(uint8_t movement, int16_t tmp_leftWheelTicks, int16_t tmp_rightWheelTicks){
  switch (movement){
	case moveForward:{
	  if(tmp_leftWheelTicks > tmp_rightWheelTicks) {
		nxt_motor_set_speed(servoLeft, SPEED-10, 1);
		nxt_motor_set_speed(servoRight, SPEED, 1);
	  } else if(tmp_rightWheelTicks > tmp_leftWheelTicks) {
		nxt_motor_set_speed(servoLeft, SPEED, 1);
		nxt_motor_set_speed(servoRight, SPEED-10, 1);
	  } else {
		nxt_motor_set_speed(servoLeft, SPEED, 1);
		nxt_motor_set_speed(servoRight, SPEED, 1);
	  }
	  break;
	}
	case moveBackward:{
	  if(tmp_leftWheelTicks < tmp_rightWheelTicks) {
		nxt_motor_set_speed(servoLeft, -SPEED+10, 1);
		nxt_motor_set_speed(servoRight, -SPEED, 1);
	  } else if(tmp_rightWheelTicks < tmp_leftWheelTicks) {
		nxt_motor_set_speed(servoLeft, -SPEED, 1);
		nxt_motor_set_speed(servoRight, -SPEED+10, 1);
	  } else {
		nxt_motor_set_speed(servoLeft, -SPEED, 1);
		nxt_motor_set_speed(servoRight, -SPEED, 1);
	  }
	  
	  break;
	}
	case moveClockwise:{
	  if(tmp_leftWheelTicks > -tmp_rightWheelTicks) {
		nxt_motor_set_speed(servoLeft, SPEED-10, 1);
		nxt_motor_set_speed(servoRight, -SPEED, 1);
	  } else if(-tmp_rightWheelTicks > tmp_leftWheelTicks) {
		nxt_motor_set_speed(servoLeft, SPEED, 1);
		nxt_motor_set_speed(servoRight, -SPEED+10, 1);
	  } else {
		nxt_motor_set_speed(servoLeft, SPEED, 1);
		nxt_motor_set_speed(servoRight, -SPEED, 1);
	  }
	  
	  break;
	}
	case moveRightForward:{
	  nxt_motor_set_speed(servoRight, SPEED, 1);
	  nxt_motor_set_speed(servoLeft, 0, 1);
	  break;
	}
	case moveLeftForward:{
	  nxt_motor_set_speed(servoLeft, SPEED, 1);
	  nxt_motor_set_speed(servoRight, 0, 1);
	  break;
	}
	case moveRightBackward:{
	  nxt_motor_set_speed(servoRight, -SPEED, 1);
	  nxt_motor_set_speed(servoLeft, 0, 1);
	  break;
	}
	case moveLeftBackward:{
	  nxt_motor_set_speed(servoLeft, -SPEED, 1);
	  nxt_motor_set_speed(servoRight, 0, 1);
	  break;
	}
	case moveCounterClockwise:{
	  if(-tmp_leftWheelTicks > tmp_rightWheelTicks) {
		nxt_motor_set_speed(servoLeft, -SPEED+10, 1);
		nxt_motor_set_speed(servoRight, SPEED, 1);
	  } else if(tmp_rightWheelTicks > -tmp_leftWheelTicks) {
		nxt_motor_set_speed(servoLeft, -SPEED, 1);
		nxt_motor_set_speed(servoRight, SPEED-10, 1);
	  } else {
		nxt_motor_set_speed(servoLeft, -SPEED, 1);
		nxt_motor_set_speed(servoRight, SPEED, 1);
	  }
	  break;
	}
	default:{
	  nxt_motor_set_speed(servoLeft, 0, 1);
	  nxt_motor_set_speed(servoRight, 0, 1);
	  break;
	}
  }
}

/// New function for use in pose controller task
void vMotorMovementSwitch(int16_t leftSpeed, int16_t rightSpeed, uint8_t *leftWheelDirection, uint8_t *rightWheelDirection) {
	if (leftSpeed > 0) {
		nxt_motor_set_speed(servoLeft, leftSpeed, leftWheelDirection);
	} else if (leftSpeed < 0) {

	} else {
		// brake left
	}

	if (rightSpeed > 0) {

	} else if (rightSpeed < 0) {

	} else {
		//brake right
	}
}

void vMotorSetAngle(uint8_t motor, int16_t angle) {
  angle = -angle; //Application code uses opposite encoder values to the nxt
  if(angle < -90) angle = -90;
  nxt_motor_command(motor, (int) floor(angle*TICKS_PER_DEGREE), TOWER_SPEED);
}

void vMotorCountUpdate(void){
  gRightWheelTicks = nxt_motor_get_count(servoRight);
  gLeftWheelTicks = nxt_motor_get_count(servoLeft);
}