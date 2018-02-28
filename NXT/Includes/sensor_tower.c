#include "sensor_tower.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "types.h"
#include "defines.h"
#include "functions.h"
#include "motor.h"
#include "io.h"
#include "communication.h"

extern volatile uint8_t gHandshook;
extern volatile uint8_t gPaused;

extern QueueHandle_t movementStatusQ;
extern QueueHandle_t globalPoseQ;
extern QueueHandle_t poseControllerQ;
extern QueueHandle_t mappingMeasurementQ;
extern TaskHandle_t xMappingTask;

/**
 * Task responsible for control of the sensor tower. Tower rotation depends on
 * movement status received from the pose controller.
 */
void vMainSensorTowerTask( void *pvParameters ) {
	/* Task init */
	pose_t Pose = { 0, 0, 0 };
	
	uint8_t rotationDirection = moveCounterClockwise;
	uint8_t servoStep = 0;
    const uint8_t SERVO_RESOLUTION = 5;
	uint8_t robotMovement = moveStop;
	uint8_t lastRobotMovement = robotMovement;
	uint8_t idleCounter = 0;
	  
	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime;
	
	while(1) {
		// Loop
		if ((gHandshook == TRUE) && (gPaused == FALSE)) {
			// xLastWakeTime variable with the current time.
			xLastWakeTime = xTaskGetTickCount();
			// Set scanning resolution depending on which movement the robot is executing.
			// Note that the iterations are skipped while robot is rotating (see further downbelow)
			if (xQueuePeek(movementStatusQ, &robotMovement, 150 / portTICK_PERIOD_MS) == pdTRUE) {
				if (robotMovement != lastRobotMovement) {
					// Tell mapping task to start line creation.
					xTaskNotifyGive(xMappingTask);
					lastRobotMovement = robotMovement;
				}

				switch (robotMovement)
				{
					case moveStop:
						//servoStep *= servoResolution;
                    	//servoResolution = 2;
                    	idleCounter = 1;
					case moveForward:
					case moveBackward:
						//servoResolution = 2;
                    	//servoStep /= servoResolution;
                    	idleCounter = 0;
                    	break;
					case moveClockwise:
					case moveCounterClockwise:
						// Iterations are frozen while rotating, see further down
						idleCounter = 0;
						break;
					default:
						idleCounter = 0;
						break;
				}
			}

			vMotorSetAngle(servoTower, servoStep);
			//vMotorSetAngle(servoTower, servoStep*servoResolution);
	  
		  	// Wait total of 200 ms for servo to reach set point and allow previous update message to be transfered.
		  	vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_PERIOD_MS);   
		  
		  	// Get measurements from sensors
		  	uint8_t forwardSensor = distance_get_cm(0);
		  	uint8_t leftSensor = distance_get_cm(1);
		  	uint8_t rearSensor = distance_get_cm(2);
		  	uint8_t rightSensor = distance_get_cm(3);

		  	// Add measurements to struct for sending to queue
		  	measurement_t Measurement = { { forwardSensor, leftSensor, rearSensor, rightSensor }, servoStep };

			// Send Measurement to mapping task
		  	xQueueSendToBack(mappingMeasurementQ, &Measurement, 10);

		  	if ((idleCounter > 10) && (robotMovement == moveStop)) {
				// If the robot stands idle for 1 second, send 'status:idle' in case the server missed it.
				send_idle();
				idleCounter = 1;
		  	}
		  	else if ((idleCounter >= 1) && (robotMovement == moveStop)) {
				idleCounter++;
		  	}

		  	#define SEND_UPDATE
		  	#ifdef SEND_UPDATE
		  	// Get the latest pose estimate, dont't remove from queue
		  	xQueuePeek(globalPoseQ, &Pose, 0);

		  	// Convert to range [0,2pi) for compatibility with server
		  	func_wrap_to_2pi(&Pose.theta);
		  
		  	//Send updates to server in the correct format (centimeter and degrees, rounded)
		  	send_update(ROUND(Pose.x/10), ROUND(Pose.y/10), ROUND(Pose.theta*RAD2DEG), servoStep, forwardSensor, leftSensor, rearSensor, rightSensor);
		  	#endif

		  	//#define MANUAL
		  	#ifndef MANUAL
		  	// Low level anti collision
		  	uint8_t objectX;
		  
		  	if ((servoStep) <= 30) objectX = forwardSensor; // * cos(servoStep*5);
		  	else if ((servoStep) >= 60) objectX = rightSensor; // * cos(270 + servoStep*5);
		  	else objectX = 0;
		  
		  	if ((objectX > 0) && (objectX < 20)) {
				// Stop controller
				//point_t Target = {0};
				//xQueuePeek(globalPoseQ, &Target, 100);
				//xQueueOverwrite(poseControllerQ, &Target); // Uses overwrite, robot must stop immediately
				xQueueReset(poseControllerQ);
				send_idle();
		  	}            
		  	#endif

		  	// Iterate in a increasing/decreasing manner and depending on the robots movement
		  	if ((servoStep <= 90) && (rotationDirection == moveCounterClockwise) && (robotMovement < moveClockwise)) {
				//servoStep++;
				//servoStep = servoStep + SERVO_RESOLUTION;
				servoStep += SERVO_RESOLUTION;
		  	} 
		  	else if ((servoStep > 0) && (rotationDirection == moveClockwise) && (robotMovement < moveClockwise)) {
				//servoStep--;
				servoStep -= SERVO_RESOLUTION;
		  	}
		  
		  	if ((servoStep >= 90) && (rotationDirection == moveCounterClockwise)) {
				rotationDirection = moveClockwise;
				// Notify mapping task about tower direction change
				xTaskNotifyGive(xMappingTask);
		  	}
		  	else if ((servoStep <= 0) && (rotationDirection == moveClockwise)) {
				rotationDirection = moveCounterClockwise;
				// Notify mapping task about tower direction change
            	xTaskNotifyGive(xMappingTask);
		  	}

		}

		else if (gHandshook == TRUE && gPaused == TRUE) {
    		vTaskDelay(200 / portTICK_PERIOD_MS);
	    }

	    else { // Disconnected or unconfirmed
		    xLastWakeTime = xTaskGetTickCount();
		  	vMotorSetAngle(servoTower, 0);
		  	//led_set(LED_YELLOW);
		  	// Reset servo incrementation
		  	rotationDirection = moveCounterClockwise;
		  	servoStep = 0;
		  	idleCounter = 0;
		  	vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_PERIOD_MS);
		  	//led_clear(LED_YELLOW);
		}
  	}
}