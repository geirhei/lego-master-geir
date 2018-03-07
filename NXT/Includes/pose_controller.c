#include "pose_controller.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <math.h>

#include "types.h"
#include "functions.h"
#include "motor.h"
#include "communication.h"

extern volatile uint8_t gHandshook;
extern volatile uint8_t gPaused;

extern QueueHandle_t globalPoseQ;
extern QueueHandle_t poseControllerQ;
extern QueueHandle_t movementStatusQ;
extern TaskHandle_t xPoseCtrlTask;

void vMainPoseControllerTask( void *pvParameters ) {
    #ifdef DEBUG
        //debug("PoseController OK\n");
        uint8_t tellar = 0;
    #endif

    /* Task init */
    point_t Target = {0};
	float radiusEpsilon = 5; //[mm]The acceptable radius from goal for completion
	uint8_t lastMovement = 0;
	
	// Find better values for NXT
	float maxRotateActuation = 40; //The max speed the motors will run at during rotation (was 75)
	float maxDriveActuation = 45; //The max speed the motors will run at during drive (was 100)
	float currentDriveActuation = maxRotateActuation;
	
	/* Controller variables for tuning, probably needs calculation for NXT */
	float rotateThreshold = 0.5235; // [rad] The threshold at which the robot will go from driving to rotation. Equals 10 degrees
	float driveThreshold = 0.0174; // [rad]The threshold at which the robot will go from rotation to driving. In degrees.
	float driveKp = 600; //Proportional gain for theta control during drive
	float driveKi = 10; //Integral gain for theta during drive
	float speedDecreaseThreshold = 300; //[mm] Distance from goal where the robot will decrease its speed inverse proportionally
	
	/* Current position variables */	
	float thetahat = 0;
	float xhat = 0;
	float yhat = 0;
	pose_t GlobalPose = {0};
	
	/* Goal variables*/
	float distance = 0;
	float thetaDiff = 0;
	float xTargt = 0;
	float yTargt = 0;
	
	float leftIntError = 0;
	float rightIntError = 0;
	
	uint8_t doneTurning = TRUE;
	
	uint8_t gLeftWheelDirection = 0;
	uint8_t gRightWheelDirection = 0;
	
	uint8_t idleSent = FALSE;
      
	while(1) {
		// Checking if server is ready
		if (gHandshook == TRUE && gPaused == FALSE) {
			
			// Wait for synchronization by direct notification from the estimator task. Timeout after
			// 1000ms to check if we are still connected.
			ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS);

			if (xQueuePeek(globalPoseQ, &GlobalPose, 0) == pdTRUE) {
				thetahat = GlobalPose.theta;
				xhat = GlobalPose.x;
				yhat = GlobalPose.y;
			}
			
			// Check if a new update is received
			if (xQueuePeek(poseControllerQ, &Target, 0) == pdTRUE) { // Read theta and radius set points from com task
				xTargt = Target.x;
				yTargt = Target.y;
			} else {
				xTargt = xhat;
				yTargt = yhat;
			}
			
			distance = sqrt((xTargt-xhat)*(xTargt-xhat) + (yTargt-yhat)*(yTargt-yhat));
			
			//Simple speed controller as the robot nears the target
			if (distance < speedDecreaseThreshold) {
				currentDriveActuation = (maxDriveActuation - 0.32 * maxDriveActuation)*distance / speedDecreaseThreshold + 0.32 * maxDriveActuation; //Reverse proportional + a constant so it reaches. 
			} else {
				currentDriveActuation = maxDriveActuation;
			}
			
			if (distance > radiusEpsilon) { //Not close enough to target
				idleSent = FALSE;
				
				float xdiff = xTargt - xhat;
				float ydiff = yTargt - yhat;
				float thetaTargt = atan2(ydiff,xdiff); //atan() returns radians
				thetaDiff = thetaTargt - thetahat; //Might be outside pi to -pi degrees
				vFunc_Inf2pi(&thetaDiff);

				//Hysteresis mechanics
				if (fabs(thetaDiff) > rotateThreshold) {
					doneTurning = FALSE;
				} else if (fabs(thetaDiff) < driveThreshold) {
					doneTurning = TRUE;
				}
				
				float LSpeed = 0;
				float RSpeed = 0;
				
				if (doneTurning) { //Start forward movement
					if (thetaDiff >= 0) { //Moving left
						LSpeed = currentDriveActuation - driveKp*fabs(thetaDiff) - driveKi*leftIntError; //Simple PI controller for theta 
						
						//Saturation
						if (LSpeed > currentDriveActuation) {
							LSpeed = currentDriveActuation;
						} else if (LSpeed < 0) {
							LSpeed = 0;
						}
						RSpeed = currentDriveActuation;
					} else { //Moving right
						RSpeed = currentDriveActuation - driveKp*fabs(thetaDiff) - driveKi*rightIntError; //Simple PI controller for theta
						
						//Saturation
						if (RSpeed > currentDriveActuation) {
							RSpeed = currentDriveActuation;
						} else if (RSpeed < 0) {
							RSpeed = 0;
						}
						LSpeed = currentDriveActuation;
					}
					
					leftIntError += thetaDiff;
					rightIntError -= thetaDiff;
					
					gRightWheelDirection = motorForward; //?
					gLeftWheelDirection = motorForward;
					lastMovement = moveForward;
					
				} else { //Turn within 1 degree of target
					if (thetaDiff >= 0) { //Rotating left
						LSpeed = -maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
						gLeftWheelDirection = motorBackward;
						RSpeed = maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
						gRightWheelDirection = motorForward;
						lastMovement = moveCounterClockwise;
					} else { //Rotating right
						LSpeed = maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
						gLeftWheelDirection = motorForward;
						RSpeed = -maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
						gRightWheelDirection = motorBackward;
						lastMovement = moveClockwise;
					}
					
					leftIntError = 0;
					rightIntError = 0;
				}

				vMotorMovementSwitch(ROUND(LSpeed), ROUND(RSpeed), &gLeftWheelDirection, &gRightWheelDirection);
		
			} else {
				if (idleSent == FALSE) {
					send_idle();
					idleSent = TRUE;
				}
				// Set speed of both motors to 0
				vMotorMovementSwitch(0, 0, &gLeftWheelDirection, &gRightWheelDirection);
				lastMovement = moveStop;
			}

			// Send the current movement to the sensor tower task
			xQueueOverwrite(movementStatusQ, &lastMovement);
			
		} else {
			// Stop motors if we get disconnected
			if (lastMovement != moveStop) {
				vMotorMovementSwitch(0, 0, &gLeftWheelDirection, &gRightWheelDirection);	
			}
			vTaskDelay(PERIOD_MOTOR_MS / portTICK_PERIOD_MS);
		}
	}
}