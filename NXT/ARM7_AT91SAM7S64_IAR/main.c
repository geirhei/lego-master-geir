/************************************************************************/
// File:            main.c
// Author:          Erlend Ese, NTNU Spring 2016
//                  Modified for use with NXT by Kristian Lien, Spring 2017
//                  Credit is given where credit is due.
// Purpose:
// NXT Robot with FreeRTOS implementation in the collaborating robots
// project.
// In FreeRTOS each thread of execution is known as tasks
// and are written as C functions
//
//
// TASKS IMPLEMENTED
// Communication:               vMainCommunicationTask
// Sensors:                     vMainSensorTowerTask
// Motor control                vMainMovementTask
// Robot control:               vMainPoseControllerTask
// Position estimator:          vMainPoseEstimatorTask
// Stack overflow handling:     vApplicationStackOverflowHook
//
// See FreeRTOSConfig.h for scheduler settings
// See defines.h for all definitions
//
/************************************************************************/

/* KERNEL INCLUDES */
#include "FreeRTOS.h" /* Must come first */
#include "task.h"     /* RTOS task related API prototypes */
#include "semphr.h"   /* Semaphore related API prototypes */
#include "queue.h"    /* RTOS queue related API prototypes */

#include <stdlib.h>         // For itoa();
#include <string.h>         // For stringstuff
#include <stdio.h>
#include <math.h>

#include "display.h"
#include "hs.h"
#include "motor.h"
#include "server_communication.h"
#include "defines.h"
#include "functions.h"
#include "nxt.h"
#include "nxt_motors.h"
#include "led.h"
#include "io.h"
#include "arq.h"
#include "simple_protocol.h"
#include "network.h"

#define TRUE    1
#define FALSE   0

/* Semaphore handles */
SemaphoreHandle_t xScanLock;
SemaphoreHandle_t xPoseMutex;
SemaphoreHandle_t xUartMutex;
SemaphoreHandle_t xTickMutex;
SemaphoreHandle_t xControllerBSem;
SemaphoreHandle_t xCommandReadyBSem;

/* Queues */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t actuationQ = 0;
QueueHandle_t driveStatusQ = 0;

/* GLOBAL VARIABLES */
// To store ticks from encoder, only changed in ISR but accessed from other
volatile int16_t gRightWheelTicks = 0;
volatile int16_t gLeftWheelTicks = 0;

// To store motor direction, only changed in motor controller, but accessed from ISR
uint8_t gLeftWheelDirection;
uint8_t gRightWheelDirection;

// Flag to indicate connection status. Interrupt can change handshook status
volatile uint8_t gHandshook = FALSE;
volatile uint8_t gPaused = FALSE;
volatile message_t message_in;

/// Task declarations
void vMainCommunicationTask( void *pvParameters );
void vMainSensorTowerTask( void *pvParameters );
void vMainPoseControllerTask( void *pvParameters );
void vARQTask( void *pvParameters );
void vMainPoseEstimatorTask( void *pvParameters );
void vMainMovementTask( void *pvParameters );
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName );

// Global robot pose
float gTheta_hat = 0;
int16_t gX_hat = 0;
int16_t gY_hat = 0;

/// Struct for storing polar coordinates
struct sPolar {
  float heading;
  int16_t distance;
};

/// Struct for storing cartesian coordinates
struct sCartesian {
	float x;
	float y;
};

#ifdef DEBUG
	#warning DEBUG IS ACTIVE
#endif

/**
 * @brief      Task responsible for processing messages received from the
 *             server. Sets global status variables and relays setpoints to the
 *             pose controller.
 *             
 *             Sends to poseControllerQ
 *
 * @param      pvParameters  The pv parameters
 */
void vMainCommunicationTask( void *pvParameters ) {
	// Setup for the communication task
	struct sPolar Setpoint = {0}; // Struct for setpoints from server

	message_t command_in; // Buffer for recieved messages

	server_communication_init();

	uint8_t success = 0;

	while (!success) {
		success = server_connect();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		led_toggle(LED_GREEN);
	}

	xTaskCreate(vARQTask, "ARQ", 500, NULL, 3, NULL);
	led_clear(LED_GREEN);
	send_handshake();

	while(1) {
		if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY)) {
			// We have a new command from the server, copy it to the memory
			vTaskSuspendAll ();       // Temporarily disable context switching
			taskENTER_CRITICAL();
			command_in = message_in;
			taskEXIT_CRITICAL();
			xTaskResumeAll ();      // Enable context switching
			  
			switch (command_in.type)
			{
				case TYPE_CONFIRM:
					taskENTER_CRITICAL();
					gHandshook = TRUE; // Set start flag true
					taskEXIT_CRITICAL();
					
					display_goto_xy(0,1);
					display_string("Connected");
					display_update();
					break;
				case TYPE_PING:
					send_ping_response();
					break;
				case TYPE_ORDER:
					Setpoint.heading = command_in.message.order.orientation;
					Setpoint.distance = command_in.message.order.distance;
					// Ensure max values are not exceeded
					if (Setpoint.distance > 320) {
						Setpoint.distance = 320;
					} else if (Setpoint.distance < -320) {
						Setpoint.distance = -320;
					}

					//
					Setpoint.distance *= 10; // Received SP is in cm, but mm is used in the controller
					// changed with new controller
					
					Setpoint.heading *= DEG2RAD; // Convert received set point to radians
					vFunc_Inf2pi(&Setpoint.heading);
					
					/* Relay new coordinates to position controller */
					xQueueSend(poseControllerQ, &Setpoint, 100);
					break;
				case TYPE_PAUSE:
					// Stop sending update messages
					taskENTER_CRITICAL();
					gPaused = TRUE;
					taskEXIT_CRITICAL();
					// Stop controller
					Setpoint.distance = 0;
					Setpoint.heading = 0;
					xQueueSend(poseControllerQ, &Setpoint, 100);
					break;
				case TYPE_UNPAUSE:
					taskENTER_CRITICAL();
					gPaused = FALSE;
					taskEXIT_CRITICAL(); 
					break;
				case TYPE_FINISH:
					taskENTER_CRITICAL();
					gHandshook = FALSE;
					taskEXIT_CRITICAL();   
					break;
			}

		}
	}
}

/**
 * @brief      Task responsible for control of the sensor tower. Tower rotation
 *             depends on movement status received from the pose controller.
 *
 *             Receives from scanStatusQ
 *             
 *             Sends to poseControllerQ (anti collision)
 *
 * @param      pvParameters  The pv parameters
 */
void vMainSensorTowerTask( void *pvParameters ) {
	/* Task init */
	float thetahat = 0;
	int16_t xhat = 0;
	int16_t yhat = 0;
	
	uint8_t rotationDirection = moveCounterClockwise;
	uint8_t servoStep = 0;
	uint8_t servoResolution = 1;
	uint8_t robotMovement = moveStop;
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
			if (xQueueReceive(scanStatusQ, &robotMovement, 150 / portTICK_PERIOD_MS) == pdTRUE) {
				switch (robotMovement)
				{
					case moveStop:
						servoStep *= servoResolution;
						servoResolution = 1;
						idleCounter = 1;
						break;
					case moveForward:
					case moveBackward:
						servoResolution = 6; // NXT-specific?
						servoStep /= servoResolution;
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
			vMotorSetAngle(servoTower, servoStep*servoResolution);
	  
			// Wait total of 200 ms for servo to reach set point and allow previous update message to be transfered.
			vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_PERIOD_MS);   
		  
			// Get measurements from sensors
			uint8_t forwardSensor = distance_get_cm(0);
			uint8_t leftSensor = distance_get_cm(1);
			uint8_t rearSensor = distance_get_cm(2);
			uint8_t rightSensor = distance_get_cm(3);
		  
			// Get latest pose estimate
			xSemaphoreTake(xPoseMutex, 40 / portTICK_PERIOD_MS);
				thetahat = gTheta_hat;
				xhat = gX_hat;
				yhat = gY_hat;
			xSemaphoreGive(xPoseMutex);
		  
			if ((idleCounter > 10) && (robotMovement == moveStop)) {
				// If the robot stands idle for 1 second, send 'status:idle' in case the server missed it.
				send_idle();
				idleCounter = 1;
			}
			else if ((idleCounter >= 1) && (robotMovement == moveStop)) {
				idleCounter++;
			}
		  
			//Send updates to server in the correct format (centimeter and degrees, rounded)
			send_update(ROUND(xhat/10), ROUND(yhat/10), ROUND(thetahat*RAD2DEG), servoStep*servoResolution, forwardSensor, leftSensor, rearSensor, rightSensor);
		  
			// Low level anti collision
			uint8_t objectX;
		  
			if ((servoStep*servoResolution) <= 30) objectX = forwardSensor; // * cos(servoStep*5);
			else if ((servoStep*servoResolution) >= 60) objectX = rightSensor; // * cos(270 + servoStep*5);
			else objectX = 0;
		  
			if ((objectX > 0) && (objectX < 20)) {
				// Stop controller
				struct sPolar Setpoint = {0, 0};
				xQueueSend(poseControllerQ, &Setpoint, 100);
			}            
		  
			// Iterate in a increasing/decreasing manner and depending on the robots movement
			if ((servoStep*servoResolution <= 90) && (rotationDirection == moveCounterClockwise) && (robotMovement < moveClockwise)) {
				servoStep++;                
			} 
			else if ((servoStep*servoResolution > 0) && (rotationDirection == moveClockwise) && (robotMovement < moveClockwise)) {
				servoStep--;
			}
		  
			if ((servoStep*servoResolution >= 90) && (rotationDirection == moveCounterClockwise)) {
				rotationDirection = moveClockwise;
			}
			else if ((servoStep*servoResolution <= 0) && (rotationDirection == moveClockwise)) {
				rotationDirection = moveCounterClockwise;
			}

		}

		else if (gPaused == TRUE && gHandshook == TRUE) {
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}

		else { // Disconnected or unconfirmed
			vMotorSetAngle(servoTower, 0);
			// Reset servo incrementation
			rotationDirection = moveCounterClockwise;
			servoStep = 0;
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}

	}
}

/**
 * @brief      Task for controlling the movement of the robot. Waits on
 *             xControllerBSem from estimator task.
 *
 * @param      pvParameters  The pv parameters
 */
void vMainPoseControllerTask( void *pvParameters ) {
  /* Task init */    
  struct sPolar Setpoint = {0}; // Struct for updates
  struct sPolar Error = {0}; // Struct for error
  struct sPolar Epsilon = {0.01745,1};
  struct sPolar oldVal = {0};
  struct sPolar referenceModel = {0};
  
  uint8_t correctHeading = TRUE;
  uint8_t correctDistance = TRUE;
  uint8_t signalEstimator = FALSE;
  uint8_t rotDir = moveStop;
  uint8_t moveDir = moveStop;
  
  float thetahat = 0;
  float integrator = 0;
  int16_t xhat = 0;
  int16_t yhat = 0;
  int16_t xInit = 0;
  int16_t yInit = 0;
  while(1){
	// Checking if server is ready
	if (gHandshook){
	  if (xSemaphoreTake(xControllerBSem, portMAX_DELAY) == pdTRUE){  
		// Wait for synchronization from estimator
		// Get robot pose
		xSemaphoreTake(xPoseMutex,portMAX_DELAY);
		thetahat = gTheta_hat;
		xhat = gX_hat;
		yhat = gY_hat;
		xSemaphoreGive(xPoseMutex);
		
		// Check if a new update is recieved
		if (xQueueReceive(poseControllerQ, &Setpoint, 0) == pdTRUE){ // Recieve theta and radius setpoints from com task, wait for 10ms if nessecary
		  // New set points, reset PID controller variables
		  oldVal.heading = 0;
		  oldVal.distance = 0;
		  integrator = 0;
		  
		  if (Setpoint.heading != 0){
			// Recieved setpoint is in (-pi,pi), use sign to decide rotation direction
			if (Setpoint.heading > 0){
			  rotDir = moveCounterClockwise;
			}
			else{
			  rotDir = moveClockwise;
			}
			Setpoint.heading += thetahat; // Adjust set point relative to current heading
			referenceModel.heading = thetahat; // Initialize reference model
			correctHeading = FALSE;
		  }
		  else if (Setpoint.heading == 0){
			correctHeading = TRUE;
			rotDir = moveStop;
			xQueueSend(movementQ, &rotDir, 0);                     
		  }
		  
		  if (Setpoint.distance != 0){
			// Use sign to decide rotation direction
			if (Setpoint.distance > 0){
			  moveDir = moveForward;
			}
			else {
			  moveDir = moveBackward;
			}
			// Initalize X and Y
			xInit = xhat;
			yInit = yhat;
			// Initialize reference model
			int32_t xSquaredCm = xhat * xhat;
			int32_t ySquaredCm = yhat * yhat;
			referenceModel.distance = ROUND(sqrt(xSquaredCm + ySquaredCm));
			correctDistance = FALSE;
			signalEstimator = TRUE;
		  }
		  else if (Setpoint.distance == 0){
			correctDistance = TRUE;
			moveDir = moveStop;
			xQueueSend(movementQ, &moveDir, 0); 
		  }
		  
		  if ((Setpoint.distance == 0) && (Setpoint.heading == 0)){
			// Signal the estimator that we have arrived at a new location
			uint8_t driveStatus = moveArrived;
			xQueueSend(driveStatusQ, &driveStatus,10);
			send_idle();
		  }
		} // if (xQueueReceive(poseControllerQ, &Setpoint, 0) == pdTRUE) end
		// No new updates from server, update position errors:
		if(correctHeading == FALSE){
		  float referenceDiff = (Setpoint.heading - referenceModel.heading);
		  vFunc_Inf2pi(&referenceDiff);
		  
		  referenceModel.heading = referenceModel.heading + (referenceDiff) / 10;
		  Error.heading = fabs(referenceModel.heading - thetahat);
		  vFunc_Inf2pi(&Error.heading);
		  
		  // Since we use cutoff to stop the robot we need to check the condition in a separate variable without the reference model.
		  float headingError = (Setpoint.heading - thetahat);
		  vFunc_Inf2pi(&headingError);
		  
		  if (fabs(headingError) < Epsilon.heading){
			rotDir = moveStop;
			correctHeading = TRUE;
			Setpoint.heading = 0;
			integrator = 0;
			xQueueSend(movementQ, &rotDir, 0);  
			if (correctDistance == TRUE){
			  send_idle();
			}
		  }
		  else{
			float dHeading = thetahat - oldVal.heading;
			vFunc_Inf2pi(&dHeading);
			
			dHeading = fabs(dHeading) / 0.030; // Divide by sample time in seconds and get positive value
			
			integrator += Error.heading;
			
			if (integrator >= 100) integrator = 100;
			
			float pidOutput = (80*Error.heading + integrator - 5*dHeading);
			
			if (pidOutput > 100) pidOutput = 100;
			else if (pidOutput < 0) pidOutput = 0;
			
			uint8_t actuation = (uint8_t)pidOutput;
			
			xQueueSend(movementQ, &rotDir, 0);
			xQueueSend(actuationQ, &actuation, 0);  
			oldVal.heading = thetahat;                     
		  }
		}
		else if (correctDistance == FALSE){
		  if (signalEstimator == TRUE){
			signalEstimator = FALSE;
			// Signal the estimator that we are going to move in a straight line
			uint8_t driveStatus = moveForward;
			xQueueSend(driveStatusQ, &driveStatus, 10);
			// Wait to be certain that the estimator has received the 
			// signal before the motor task starts
			vTaskDelay(100 / portTICK_PERIOD_MS);
		  }
		  int32_t diffX = (xInit - xhat);
		  int32_t diffY = (yInit - yhat);
		  
		  diffX *= diffX;
		  diffY *= diffY;
		  referenceModel.distance = referenceModel.distance + (abs(Setpoint.distance) - referenceModel.distance) / 10;
		  Error.distance = referenceModel.distance - ROUND(sqrt((diffX + diffY)));
		  
		  // Since we use cutoff to stop the robot we need to check the condition in a separate variable without the reference model.
		  int16_t errorDistance = abs(Setpoint.distance) - ROUND(sqrt((diffX + diffY)));
		  
		  if (errorDistance <= Epsilon.distance){
			moveDir = moveStop;
			correctDistance = TRUE;
			Setpoint.distance = 0;
			integrator = 0;
			xQueueSend(movementQ, &moveDir, 10);
			// Signal the estimator that we have arrived at a new location
			uint8_t driveStatus = moveArrived;
			xQueueSend(driveStatusQ, &driveStatus,10);   
			send_idle();              
		  }
		  else{
			int16_t dXY = ROUND((sqrt((diffX + diffY)) - oldVal.distance) / 0.030);
			
			integrator += Error.distance/5;
			if (integrator >= 100) integrator = 100;
			
			
			int16_t pidOutput = 1*Error.distance + (int16_t)integrator - dXY;
			
			if (pidOutput > 100) pidOutput = 100;
			else if (pidOutput < 0) pidOutput = 0;
			
			uint8_t actuation = (uint8_t)pidOutput;
			
			xQueueSend(movementQ, &moveDir, 0);
			xQueueSend(actuationQ, &actuation, 0); 
			oldVal.distance = ROUND(sqrt((diffX + diffY)));
		  }
		}
	  } // No semaphore available, task is blocking
	} //if(gHandshook) end
	else{
	  // Reset controller
	  correctHeading = TRUE;
	  correctDistance = TRUE;
	  moveDir = moveStop;
	  xQueueSend(movementQ, &moveDir, 200);
	  vTaskDelay(100 / portTICK_PERIOD_MS);
	}
  }// while (1)
}

/**
 * @brief      Estimates the pose of the robot using a kalman filter based
 *             algorithm. Signals the pose controller when ready.
 *
 * @param      pvParameters  The pv parameters
 */
void vMainPoseEstimatorTask( void *pvParameters ) {
	int16_t previous_ticksLeft = 0;
	int16_t previous_ticksRight = 0;

	const TickType_t xDelay = PERIOD_ESTIMATOR_MS;
    float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0f;

	float kalmanGain = 0.5;

	float predictedTheta = 0.0;
	float predictedX = 0.0;
	float predictedY = 0.0;

	float gyroOffset = 0.0;
	float compassOffset = 0.0;

	// These may need to be updated when exploring a new area
	// Can be found by using the calibration task and using
	// define COMPASS_CALIBRATE
	const int16_t xComOff = -278; 
	const int16_t yComOff = 13;

	const float variance_gyro = 0.0482f; // [rad] calculated offline, see report
	const float variance_encoder = (2.0f * WHEEL_FACTOR_MM) / (WHEELBASE_MM); // approximation, 0.0257 [rad]

	const float variance_gyro_encoder = (variance_gyro + variance_encoder) * period_in_S; // (Var gyro + var encoder) * timestep
	double covariance_filter_predicted = 0;

	#define CONST_VARIANCE_COMPASS 0.0349f // 2 degrees in rads, as specified in the data sheet (NXT)
	#define COMPASS_FACTOR 10000.0f// We are driving inside with a lot of interference, compass needs to converge slowly

	float gyroWeight = 0.5; // encoderError / (encoderError + gyroError);
	uint8_t robot_is_turning = 0;

	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1) {
		// Loop
		vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS);   

		if (gHandshook) { // Check if we are ready   
	  		int16_t leftWheelTicks = 0;
	  		int16_t rightWheelTicks = 0;

	  		// Get encoder data, protect the global tick variables.
			xSemaphoreTake(xTickMutex, 15 / portTICK_PERIOD_MS);
				leftWheelTicks = gLeftWheelTicks;
		  		rightWheelTicks = gRightWheelTicks;
		  	xSemaphoreGive(xTickMutex);

		  	float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
		  	float dRight = (float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample

		  	previous_ticksLeft = leftWheelTicks;
		  	previous_ticksRight = rightWheelTicks;
		  
		  	float dRobot = (dLeft + dRight) / 2; // Distance robot has travelled since last sample
		  	float dTheta = (dRight - dLeft) / WHEELBASE_MM; // [RAD] Get angle from encoders, derived from arch of circles formula
		  
		  	/* PREDICT */
		  	// Get gyro data:
		  	float gyrZ = (gyro_get_dps_z() - gyroOffset);

		  	// If the robot is not really rotating we don't include the gyro measurements, to avoid the trouble with drift while driving in a straight line
		  	if (fabs(gyrZ) < 10) { 
				gyroWeight = 0; // Disregard gyro while driving in a straight line
				robot_is_turning = FALSE; // Don't update angle estimates
		  	} else {
				robot_is_turning = TRUE;                
				gyroWeight = 0.85; // Found by experiment, after 20x90 degree turns, gyro seems 85% more accurate than encoders    
		  	}      
		  	
		  	// Scale gyro measurement
		  	gyrZ *= period_in_S * DEG2RAD;
		  
		  	// Fuse heading from sensors to predict heading:
		  	dTheta = (1 - gyroWeight) * dTheta + gyroWeight * gyrZ;
		  
		  	// Estimate global X and Y pos
		  	predictedX = predictedX + (dRobot * cos(predictedTheta + 0.5 * dTheta));
		  	predictedY = predictedY + (dRobot * sin(predictedTheta + 0.5 * dTheta));

		  	// Predicted (a priori) state estimate for theta
		  	predictedTheta += dTheta;

		  	// Predicted (a priori) estimate covariance
		  	covariance_filter_predicted += variance_gyro_encoder;
		  
		  	/* UPDATE */
		  	// Get compass data:
		  	int16_t xCom, yCom, zCom;
		  	compass_get(&xCom, &yCom, &zCom);
		  	// Add calibrated bias
		  	xCom += xComOff;
		  	yCom += yComOff;
		  	// calculate heading
		  	float compassHeading;
		  	compassHeading = atan2(xCom, yCom) - compassOffset ; // returns -pi, pi
		  	// Update predicted state:
		  	float error = (compassHeading - predictedTheta);
		  	vFunc_Inf2pi(&compassHeading);
		  
			// kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
		  	if (fabs(error) > (0.8727*period_in_S)) { // 0.8727 rad/s is top speed while turning
                // If we have a reading over this, we can safely ignore the compass
                // Ignore compass while driving in a straight line
                kalmanGain = 0;
            	led_clear(LED_YELLOW);
            } else if ((robot_is_turning == FALSE) && (dRobot == 0)) {
                // Updated (a posteriori) state estimate
                kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
                led_set(LED_YELLOW);
            } else {
                kalmanGain = 0;
                led_clear(LED_YELLOW);
            }

            predictedTheta += kalmanGain*(error);
			vFunc_Inf2pi(&predictedTheta);          

			// Updated (a posteriori) estimate covariance
            covariance_filter_predicted = (1 - kalmanGain) * covariance_filter_predicted; 

            // Update pose
            xSemaphoreTake(xPoseMutex, 15 / portTICK_PERIOD_MS);
                gTheta_hat = predictedTheta;
                gX_hat = predictedX;
                gY_hat = predictedY;
            xSemaphoreGive(xPoseMutex);

            // Notify the controller by semaphore
            xSemaphoreGive(xControllerBSem);

		}

		else {
		  	// Not connected, getting heading and gyro bias
		  	uint16_t i = 0;
		  	uint16_t samples = 500;
		  	float gyro = 0;
		  	for (i = 0; i <= (samples-1); i++) {
				gyro += gyro_get_dps_z();
		  	}
		  
		  	int16_t xCom, yCom, zCom = 0;
		  	compass_get(&xCom, &yCom, &zCom);       
		  	xCom += xComOff;
		  	yCom += yComOff; 
		  
		  	// Initialize pose to 0 and reset offset variables
		  	predictedX = 0;
		  	predictedY = 0;
		  	predictedTheta = 0;
		  
		  	compassOffset = atan2(xCom,yCom);
		  	gyroOffset = gyro / (float)i;
		}
	}
}

/* Handles request from position controller and sets motor pins. */
/* Frequency set by PERIOD_MOTOR_MS in defines.h */
void vMainMovementTask( void *pvParameters ){
  /* Task init */
  uint8_t lastMovement = 0;
  uint8_t movement = 0;
  uint8_t actuation = 0;
  
  int16_t bias_LeftWheelTick = 0;
  int16_t bias_RightWheelTick = 0;
  
  while(1){
	if (gHandshook){ // Check if we are connected and good to go
	  xQueueReceive(movementQ, &movement, 0);
	  xQueueReceive(actuationQ, &actuation, 0);
	  if (movement != lastMovement){
		taskENTER_CRITICAL();
		bias_LeftWheelTick = gLeftWheelTicks;
		bias_RightWheelTick = gRightWheelTicks;
		taskEXIT_CRITICAL();
		lastMovement = movement;
		xQueueSend(scanStatusQ, &lastMovement, 0);
	  }
	  int16_t tmp_leftWheelTicks = 0;
	  int16_t tmp_rightWheelTicks = 0;
	  taskENTER_CRITICAL();
	  tmp_leftWheelTicks = gLeftWheelTicks - bias_LeftWheelTick;
	  tmp_rightWheelTicks = gRightWheelTicks - bias_RightWheelTick;
	  taskEXIT_CRITICAL();
	  /* Saturate values */
	  if (actuation < 40) actuation = 40;
	  else if (actuation > 100) actuation = 100;
	  
	  /* Change actuation to millis with period defined in defines.h */
	  actuation = PERIOD_MOTOR_MS * actuation / 100;
	  if (actuation != 0){
		vMotorMovementSwitch(movement, tmp_leftWheelTicks, tmp_rightWheelTicks);
		/* ON time duration */
		vTaskDelay(actuation / portTICK_PERIOD_MS);
		/* We dont want to move */
		if (actuation != PERIOD_MOTOR_MS){
		  nxt_motor_set_speed(servoLeft, 0, 1);
		  nxt_motor_set_speed(servoRight, 0, 1);
		}
		else{/* Full throttle*/
		}
		/* Off time duration */
		vTaskDelay((PERIOD_MOTOR_MS - actuation) / portTICK_PERIOD_MS);
	  }
	  else {/*actuation is 0, do nothing*/
		vTaskDelay(20 / portTICK_PERIOD_MS);
	  }
	}
	else{ // Not connected, stop & do nothing
	  nxt_motor_set_speed(servoLeft, 0, 1);
	  nxt_motor_set_speed(servoRight, 0, 1);
	  vTaskDelay(100 / portTICK_PERIOD_MS);
	}
  }// While(1) end
}

//#define COMPASS_CALIBRATE

#ifdef COMPASS_CALIBRATE
void compassTask(void *par);

/**
 * @brief      Task for compass calibration
 *
 * @param      par   The par
 */
void compassTask(void *par ) {
  vTaskDelay(100 / portTICK_PERIOD_MS);

  int16_t xComOff = 0;
  int16_t yComOff = 0;

  while(1){
	vTaskDelay(5000 / portTICK_PERIOD_MS);
	if (gHandshook){
	  display_goto_xy(0,6);
	  display_string("Starting...");
	  display_update();
	  int16_t xComMax = -4000, yComMax = -4000;
	  int16_t xComMin = 4000, yComMin = 4000;
	  int16_t xCom, yCom, zCom;   
	  // wait until you start moving     
	  //         while(fabs(zGyr) < 20){
	  //             zGyr = fIMU_readFloatGyroZ();
	  //             
	  //             vTaskDelay(15/portTICK_PERIOD_MS);
	  //         }
	  
	  uint8_t movement;
	  movement = moveCounterClockwise;
	  xQueueSend(movementQ, &movement, 10);

	  float heading = 0;
	  float gyroHeading = 0;
	  float encoderHeading = 0;
	  
	  gLeftWheelTicks = 0;
	  gRightWheelTicks = 0;
	  float previous_ticksLeft = 0;
	  float previous_ticksRight = 0;
	  // Storing values for printing later
	  //         uint8_t tellar = 0;
	  //         float tabellG[200];
	  //         float tabellE[200];
	  
	  TickType_t xLastWakeTime;
	  const TickType_t xDelay = 50;
	  // Initialise the xLastWakeTime variable with the current time.
	  xLastWakeTime = xTaskGetTickCount(); 
	  while(heading < 359){
		vTaskDelayUntil(&xLastWakeTime, xDelay);
		int16_t leftWheelTicks = 0;
		int16_t rightWheelTicks = 0;
		taskENTER_CRITICAL();
		leftWheelTicks = gLeftWheelTicks;
		rightWheelTicks = gRightWheelTicks;
		taskEXIT_CRITICAL();
		float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
		float dRight =(float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
		previous_ticksLeft = leftWheelTicks;
		previous_ticksRight = rightWheelTicks;
		float dTheta = RAD2DEG * (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
		
		float zGyr = 0.05*(gyro_get_dps_z()+2.05); // add offset bias
		// Storing values for printing            
		//             gyroHeading += zGyr;
		//             encoderHeading += dTheta;
		//             tabellE[tellar] = dTheta;
		//             tabellG[tellar] = zGyr;
		//             tellar++;

		heading += 0.5 * zGyr  + 0.5 * dTheta; // Complementary filter

		compass_get(&xCom, &yCom, &zCom);
		
		if(xCom > xComMax) xComMax = xCom;
		if(yCom > yComMax) yComMax = yCom;
		
		if(xCom < xComMin) xComMin = xCom;
		if(yCom < yComMin) yComMin = yCom;
	  }
	  movement = moveClockwise;
	  xQueueSend(movementQ, &movement, 10);
	  movement = moveStop;
	  xQueueSend(movementQ, &movement, 10);
	  // Printing said values
	  //         int i = 0;
	  //         for (i = 0; i < tellar; i++){
	  //             printf("%.1f, %.1f\n",tabellG[i], tabellE[i]);
	  //             vTaskDelay(200 / portTICK_PERIOD_MS);
	  //         }
	  //         printf("gyro %.2f, encoder %.2f \n", gyroHeading, encoderHeading);
	  
	  // Printing new xy cal values
	  display_goto_xy(0,1);
	  display_string("Old values: ");
	  display_goto_xy(0,2);
	  display_int(xComOff, 5);
	  display_int(yComOff, 5);
	  
	  xComOff = ((xComMax - xComMin)/2) - xComMax;
	  yComOff = ((yComMax - yComMin)/2) - yComMax;
	  
	  display_goto_xy(0,3);
	  display_string("New values:");
	  display_goto_xy(0,4);
	  display_int(xComOff, 5);
	  display_int(yComOff, 5);
	  display_update();
	  vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
	else
	  vTaskDelay(200 / portTICK_PERIOD_MS); 
  }
}
#endif

#ifdef SENSOR_CALIBRTION
void vSensorCalibrationTask(void *pvParams);

/**
 * @brief      Task for sensor calibration
 *
 * @param      pvParams  The pv parameters
 */
void vSensorCalibrationTask(void *pvParams) {
  uint8_t calibration[256] = {0};
  uint8_t current_distance = 10;
  uint8_t count = 0;

  vTaskDelay(5000/portTICK_PERIOD_MS);
  while(1) {
	
	led_set(LED_GREEN);
	vTaskDelay(100/portTICK_PERIOD_MS);
	led_clear(LED_GREEN);
	vTaskDelay(100/portTICK_PERIOD_MS);

	calibration[ distance_get_volt(3) ] = current_distance;
	display_goto_xy(0,3);
	display_int(++count, 3);
	display_update();
	display_goto_xy(0,2);
	display_int(current_distance, 2);
	display_update();

	current_distance++;
	if(current_distance == 81) {
	  led_set(LED_RED);
	  break;
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  
  uint16_t i;
  uint8_t last = 0;
  for(i=0;i<255;i++) {
	debug("%d,", calibration[i] == 0 ? last : calibration[i] );
	if(calibration[i] != 0) last = calibration[i];
	vTaskDelay(100/portTICK_PERIOD_MS);
  }

  while(1) {};
}
#endif

/**
 * @brief      In case of stack overflow, disable all interrupts and handle it.
 *
 * @param      pxTask      The px task
 * @param      pcTaskName  The task name
 */
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
  led_set(LED_GREEN);
  led_set(LED_YELLOW);
  led_set(LED_RED);
  
  while(1){
  }// While(1) end
}

/**
 * @brief      The main function.
 *
 * @return     { description_of_the_return_value }
 */
int main(void){
  nxt_init();
  network_init();
  arq_init();
  simple_p_init(server_receiver);

  // Set red LED on to indicate INIT is ongoing
  led_set(LED_RED); 

  /* Initialize RTOS utilities  */
  movementQ = xQueueCreate(1,sizeof(uint8_t)); // For sending movements to vMainMovementTask
  poseControllerQ = xQueueCreate(1, sizeof(struct sPolar)); // For setpoints to controller
  scanStatusQ = xQueueCreate(1,sizeof(uint8_t)); // For robot status
  actuationQ = xQueueCreate(1,sizeof(uint8_t)); // To send variable actuations to motor task
  driveStatusQ = xQueueCreate(1,sizeof(uint8_t)); // To send if robot is driving to the estimator
  
  xPoseMutex = xSemaphoreCreateMutex(); // Global variables for robot pose. Only updated from estimator, accessed from many
  xUartMutex = xSemaphoreCreateMutex(); // Protected printf with a mutex, may cause fragmented bytes if higher priority task want to print as well
  xTickMutex = xSemaphoreCreateMutex(); // Global variable to hold robot tick values
  
  xControllerBSem = xSemaphoreCreateBinary(); // Estimator to Controller synchronization
  xCommandReadyBSem = xSemaphoreCreateBinary(); 
  BaseType_t ret;
  xTaskCreate(vMainMovementTask, "Movement", 500, NULL, 4, NULL);  // Independent task
  xTaskCreate(vMainCommunicationTask, "Comm", 1000, NULL, 3, NULL);  // Dependant on IO, sends instructions to other tasks
#ifndef COMPASS_CALIBRATE
  xTaskCreate(vMainPoseControllerTask, "PoseCon", 500, NULL, 2, NULL);// Dependant on estimator, sends instructions to movement task
  xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 500, NULL, 5, NULL); // Independent task,
  ret = xTaskCreate(vMainSensorTowerTask,"Tower", 500, NULL, 1, NULL); // Independent task, but use pose updates from estimator
#endif
  if(ret != pdPASS) {
	display_goto_xy(0,2);
	display_string("Error");
	display_update();
  }
  
#ifdef COMPASS_CALIBRATE
  display_goto_xy(0,1);
  display_string("\n \t WARNING \t !\n");
  display_string("COMPASS CALIBRATION!\n");
  display_string("{S,CON} to begin\n");
  xTaskCreate(compassTask, "compasscal", 8000, NULL, 4, NULL); // Task used for compass calibration, dependant on communication and movement task
#endif
  
  led_clear(LED_RED);
  
  /*  Start scheduler */
  display_goto_xy(0,0);
  display_string("Init complete");
  display_update();

  vTaskStartScheduler();
  
  /*  MCU is out of RAM if the program comes here */
  while(1){
	led_toggle(LED_GREEN);
	led_toggle(LED_RED);
	led_toggle(LED_YELLOW);
  }
}