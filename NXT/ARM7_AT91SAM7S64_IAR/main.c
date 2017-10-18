/************************************************************************/
// File:			main.c
// Author:			Erlend Ese, NTNU Spring 2016
//                  Modified for use with NXT by Kristian Lien, Spring 2017
//                  Updated with the Arduino's positioning algorithm by Geir Eikeland, Fall 2017
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

/* Semaphore handles */
//SemaphoreHandle_t xPoseMutex;
//SemaphoreHandle_t xUartMutex;
//SemaphoreHandle_t xTickMutex;
SemaphoreHandle_t xCommandReadyBSem;
SemaphoreHandle_t xCollisionBSem;

/* Queues */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t globalWheelTicksQ = 0;
QueueHandle_t globalPoseQ = 0;
QueueHandle_t priorityOrderQ = 0;

/* Task handles */
TaskHandle_t xPoseCtrlTask = NULL;

/* GLOBAL VARIABLES */
// To store ticks from encoder, changed in ISR and motor controller
/*
volatile uint8_t gISR_rightWheelTicks = 0;
volatile uint8_t gISR_leftWheelTicks = 0;
*/

// Global encoder tick values. Replaced by globalWheelTicksQ
/*
volatile int16_t gRightWheelTicks = 0;
volatile int16_t gLeftWheelTicks = 0;
*/

// Flag to indicate connection status.
volatile uint8_t gHandshook = FALSE;
volatile uint8_t gPaused = FALSE;
volatile message_t message_in;

/// Task declarations
void vMainCommunicationTask( void *pvParameters );
void vMainSensorTowerTask( void *pvParameters );
void vMainPoseControllerTask( void *pvParameters );
void vARQTask( void *pvParameters );
void vMainPoseEstimatorTask( void *pvParameters );
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );

/// Struct for storing wheel ticks
struct sWheelTicks {
	int16_t rightWheel;
	int16_t leftWheel;
};

// Global robot pose
/*
float gTheta_hat = 0;
int16_t gX_hat = 0;
int16_t gY_hat = 0;
*/

// Struct for storing robot pose
struct sPose {
	float theta;
	float x;
	float y;
};

/// Struct for storing polar coordinates
struct sPolar {
  int16_t heading;
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
	//struct sPolar Setpoint = {0}; // Struct for setpoints from server
	struct sCartesian Target = {0}; // Structs for target coordinates from server
	struct sPolar PriorityOrder = {0};
	message_t command_in; // Buffer for recieved messages

	server_communication_init();

	uint8_t success = 0;
	while (!success) {
		success = server_connect();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		led_toggle(LED_GREEN);
	}

	xTaskCreate(vARQTask, "ARQ", 250, NULL, 3, NULL);
	led_clear(LED_GREEN);
	send_handshake();

	while(1) {
		if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY) == pdTRUE) {
			// We have a new command from the server, copy it to the memory
			vTaskSuspendAll ();       // Temporarily disable context switching
			taskENTER_CRITICAL();
			command_in = message_in;
			taskEXIT_CRITICAL();
			xTaskResumeAll();      // Enable context switching
			  
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
				case TYPE_PRIORITY_ORDER:
					// Pass the received order to the priorityOrderQ
					PriorityOrder.heading = command_in.message.priority_order.heading;
					PriorityOrder.distance = command_in.message.priority_order.distance;
					xQueueOverwrite(priorityOrderQ, &PriorityOrder);
					break;
				case TYPE_ORDER:
					/*
					Setpoint.heading = command_in.message.order.orientation;
					Setpoint.distance = command_in.message.order.distance;
					// Ensure max values are not exceeded
					if (Setpoint.distance > 320) {
						Setpoint.distance = 320;
					} else if (Setpoint.distance < -320) {
						Setpoint.distance = -320;
					}
					
					Setpoint.heading *= DEG2RAD; // Convert received set point to radians
					vFunc_Inf2pi(&Setpoint.heading);
					*/

					Target.x = (float) command_in.message.order.x;
					Target.y = (float) command_in.message.order.y;

					//debug("%iTarget x: ", Target.x);
					//debug("%iTarget y: ", Target.y);

					/* Relay new coordinates to position controller */
					xQueueOverwrite(poseControllerQ, &Target);
					break;
				case TYPE_PAUSE:
					//led_set(LED_YELLOW);
					// Stop sending update messages
					taskENTER_CRITICAL();
					gPaused = TRUE;
					taskEXIT_CRITICAL();
					// Stop controller - pass the current position
					xQueuePeek(globalPoseQ, &Target, 1);
					xQueueOverwrite(poseControllerQ, &Target);
					break;
				case TYPE_UNPAUSE:
					//led_set(LED_RED);
					taskENTER_CRITICAL();
					gPaused = FALSE;
					taskEXIT_CRITICAL(); 
					//led_clear(LED_RED);
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

	struct sPose globalPose = {0};
	
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

				//debug("%d", robotMovement);
				switch (robotMovement)
				{
					case moveStop:
						servoStep *= servoResolution;
						servoResolution = 1;
						idleCounter = 1;
						break;
					case moveForward:
					case moveBackward:
						servoResolution = 1;
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
				//debug("%d", servoStep);
			}

			vMotorSetAngle(servoTower, servoStep*servoResolution);
	  
		  	// Wait total of 200 ms for servo to reach set point and allow previous update message to be transfered.
		  	vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_PERIOD_MS);   
		  
		  	// Get measurements from sensors
		  	uint8_t forwardSensor = distance_get_cm(0);
		  	uint8_t leftSensor = distance_get_cm(1);
		  	uint8_t rearSensor = distance_get_cm(2);
		  	uint8_t rightSensor = distance_get_cm(3);
		  
		  	/*
		  	xSemaphoreTake(xPoseMutex, 40 / portTICK_PERIOD_MS);
		  		thetahat = gTheta_hat;
		  		xhat = gX_hat;
		  		yhat = gY_hat;
		  	xSemaphoreGive(xPoseMutex);
			*/

		  	// Get latest pose estimate, dont't remove from queue
		  	if (xQueuePeek(globalPoseQ, &globalPose, 40 / portTICK_PERIOD_MS)) {
		  		thetahat = globalPose.theta;
		  		xhat = globalPose.x;
		  		yhat = globalPose.y;
		  	}
		  
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
				//struct sPolar Setpoint = {0, 0};
				struct sCartesian Target = {0};
				xQueuePeek(globalPoseQ, &Target, 100);
				//xQueueSend(poseControllerQ, &Setpoint, 100);
				xQueueOverwrite(poseControllerQ, &Target); // Uses overwrite, must stop immediately
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

/*  Calculates new settings for the movement task */
void vMainPoseControllerTask( void *pvParameters ) {
    #ifdef DEBUG
        printf("PoseController OK\n");
        uint8_t tellar = 0;
    #endif

    /* Task init */
    //struct sPolar Setpoint = {0}; // Updates from server
    struct sCartesian Target = {0};
    //struct sCartesian Error = {0}; // Error values
    //struct sPolar oldVal = {0};
    //struct sPolar referenceModel = {0};
	float radiusEpsilon = 15; //[mm]The acceptable radius from goal for completion
	uint8_t lastMovement = 0;
	
	// Find better values for NXT
	uint8_t maxRotateActuation = 40; //The max speed the motors will run at during rotation (was 75)
	uint8_t maxDriveActuation = 45; //The max speed the motors will run at during drive (was 100)
	uint8_t currentDriveActuation = maxRotateActuation;
	
	/* Controller variables for tuning, probably needs calculation for NXT */
	float rotateThreshold = 0.5235; // [rad] The threshold at which the robot will go from driving to rotation. Equals 10 degrees
	float driveThreshold = 0.0174; // [rad]The threshold at which the robot will go from rotation to driving. In degrees.
	float driveKp = 600; //Proportional gain for theta control during drive
	float driveKi = 10; //Integral gain for theta during drive
	float speedDecreaseThreshold = 500; //[mm] Distance from goal where the robot will decrease its speed inverse proportionally
	
	/* Current position variables */	
	float thetahat = 0;
	float xhat = 0;
	float yhat = 0;
	struct sPose GlobalPose = {0};
	
	/* Goal variables*/
	float distance = 0;
	//float thetaDiff = 0;
	float xTargt = 0;
	float yTargt = 0;
	
	//float prevLeftActuation = 0;
	//float prevRightActtion = 0;
	float leftIntError = 0;
	float rightIntError = 0;
	
	uint8_t doneTurning = TRUE;
	
	int16_t leftWheelTicks = 0;
	int16_t rightWheelTicks = 0;
	struct sWheelTicks WheelTicks = {0};
	
	uint8_t leftEncoderVal = 0;
	uint8_t rightEncoderVal = 0;
	
	uint8_t gLeftWheelDirection = 0;
	uint8_t gRightWheelDirection = 0;
	
	uint8_t idleSent = FALSE;
      
	while(1) {
		// Checking if server is ready
		if (gHandshook) {
			
			vMotorEncoderLeftTickFromISR(gLeftWheelDirection, &leftWheelTicks, leftEncoderVal);
			vMotorEncoderRightTickFromISR(gRightWheelDirection, &rightWheelTicks, rightEncoderVal);
			
			WheelTicks.leftWheel = leftWheelTicks;
			WheelTicks.rightWheel = rightWheelTicks;

			// Send wheel ticks received from ISR to the global wheel tick Q. Wait 1ms - increase this?
			xQueueOverwrite(globalWheelTicksQ, &WheelTicks);
			
			// Wait for synchronization by direct notification from the estimator task. Blocks indefinetely?
			ulTaskNotifyTake(pdTRUE, 100 / portTICK_PERIOD_MS);

			if (xQueuePeek(globalPoseQ, &GlobalPose, 100 / portTICK_PERIOD_MS)) { // careful with the portmax_delay here
				thetahat = GlobalPose.theta;
				xhat = GlobalPose.x;
				yhat = GlobalPose.y;

				//debug("%f", thetahat);
				//debug("%f", yhat);
			}
			
			// Check if a new update is received
			if (xQueueReceive(poseControllerQ, &Target, 20 / portTICK_PERIOD_MS)) { // Receive theta and radius set points from com task, wait for 20ms if necessary
				//Setpoint.distance = Setpoint.distance*10; //Distance is received in cm, convert to mm for continuity
				//xTargt = xhat + Setpoint.distance*cos(Setpoint.heading + thetahat);
				//yTargt = yhat + Setpoint.distance*sin(Setpoint.heading + thetahat);

				// Coordinates received in cm, convert to mm for continuity
				xTargt = Target.x*10;
				yTargt = Target.y*10;

				//debug("%f", xTargt);
				//debug("%f", yTargt);
			}
			
			distance = sqrt((xTargt-xhat)*(xTargt-xhat) + (yTargt-yhat)*(yTargt-yhat));
			//debug("%f", distance);
			
			//Simple speed controller as the robot nears the target
			if (distance < speedDecreaseThreshold) {
				currentDriveActuation = (maxDriveActuation - 0.32*maxDriveActuation)*distance/speedDecreaseThreshold + 0.32*maxDriveActuation; //Reverse proportional + a constant so it reaches. 
			} else {
				currentDriveActuation = maxDriveActuation;
			}
			
			if (distance > radiusEpsilon) { //Not close enough to target
				idleSent = FALSE;
				
				float xdiff = xTargt - xhat;
				float ydiff = yTargt - yhat;
				float thetaTargt = atan2(ydiff,xdiff); //atan() returns radians
				float thetaDiff = thetaTargt - thetahat; //Might be outside pi to -pi degrees
				vFunc_Inf2pi(&thetaDiff);
				
				//Hysteresis mechanics
				if (fabs(thetaDiff) > rotateThreshold) {
					doneTurning = FALSE;
				} else if (fabs(thetaDiff) < driveThreshold) {
					doneTurning = TRUE;
				}
				
				int16_t LSpeed = 0;
				int16_t RSpeed = 0;
				
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
				
				vMotorMovementSwitch(LSpeed, RSpeed, &gLeftWheelDirection, &gRightWheelDirection);
		
			} else {
				if (idleSent == FALSE) {
					send_idle();
					idleSent = TRUE;
				}
				
				vMotorBrakeLeft();
				vMotorBrakeRight();
				lastMovement = moveStop;
			}

			xQueueSend(scanStatusQ, &lastMovement, 0); // Send the current movement to the sensor tower task
			
		//} // No semaphore available, task is blocking
		} //if(gHandshook) end
	}
}

/* Pose estimator task */
// New values and constants should be calibrated for the NXT
void vMainPoseEstimatorTask( void *pvParameters ) {
    int16_t previous_ticksLeft = 0;
    int16_t previous_ticksRight = 0;  
    
    const TickType_t xDelay = PERIOD_ESTIMATOR_MS;
    float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0f;
    
    float kalmanGain = 0.5;
    
    float predictedTheta = 0.5*M_PI; // Start-heading i 90 degrees
    float predictedX = 0.0;
    float predictedY = 0.0;
    struct sPose PredictedPose = {0};
    
    float gyroOffset = 0.0;
    float compassOffset = 0.0;
    
    // Found by using calibration task
    int16_t xComOff = 11; 
    int16_t yComOff = -78;
    
    float variance_gyro = 0.0482f; // [rad] calculated offline, see report
    float variance_encoder = (2.0f * WHEEL_FACTOR_MM) / (WHEELBASE_MM); // approximation, 0.0257 [rad]
    
    float variance_gyro_encoder = (variance_gyro + variance_encoder) * period_in_S; // (Var gyro + var encoder) * timestep
    float covariance_filter_predicted = 0;
    
    #define CONST_VARIANCE_COMPASS 0.0349f // 2 degrees in rads, as specified in the data sheet
	#define COMPASS_FACTOR 10000.0f// We are driving inside with a lot of interference, compass needs to converge slowly
    
    float gyroWeight = 0.5;//encoderError / (encoderError + gyroError);
    uint8_t robot_is_turning = 0;
    
    #ifdef DEBUG
        printf("Estimator OK");
        printf("[%i]",PERIOD_ESTIMATOR_MS);
        printf("ms\n");   
        uint8_t printerTellar = 0;     
    #endif
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        // Loop
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );
        if (gHandshook) { // Check if we are ready    
            int16_t leftWheelTicks = 0;
            int16_t rightWheelTicks = 0;
            struct sWheelTicks WheelTicks = {0};

            // Attempt to receive global tick data, move on after 15ms
            if (xQueueReceive(globalWheelTicksQ, &WheelTicks, 15 / portTICK_PERIOD_MS)) {
            	leftWheelTicks = WheelTicks.leftWheel;
            	rightWheelTicks = WheelTicks.rightWheel;
            }
            
            float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
            float dRight = (float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
            
            previous_ticksLeft = leftWheelTicks;
            previous_ticksRight = rightWheelTicks;
					   
            float dRobot = (dLeft + dRight) / 2;  
            float dTheta = (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
            
            /* PREDICT */
            // Get gyro data:
            float gyrZ = (gyro_get_dps_z() - gyroOffset);
            
            // If the robot is not really rotating we don't include the gyro measurements, to avoid the trouble with drift while driving in a straight line
            if (fabs(gyrZ) < 10) {
            	gyroWeight = 0; // Disregard gyro while driving in a straight line
				robot_is_turning = FALSE; // Don't update angle estimates
			} else {
                gyroWeight = 0.75; // Found by experiment, after 20x90 degree turns, gyro seems 85% more accurate than encoders    
                robot_is_turning = TRUE;
            }
            
            // Scale gyro measurement
            gyrZ *= period_in_S * DEG2RAD;
            
            // Fuse heading from sensors to predict heading:
            dTheta = (1 - gyroWeight) * dTheta + gyroWeight * gyrZ;
            
            // Estimate global X and Y pos
            // Todo; Include accelerator measurements to estimate position and handle wheel slippage
            predictedX = predictedX + (dRobot * cos(predictedTheta + 0.5 * dTheta)); 
            predictedY = predictedY + (dRobot * sin(predictedTheta + 0.5 * dTheta));

            // Predicted (a priori) state estimate for theta
            predictedTheta += dTheta;
                  
            // Predicted (a priori) estimate covariance
            covariance_filter_predicted += variance_gyro_encoder;
            
            /* UPDATE */
            // Get compass data: ( Request and recheck after 6 ms?)
            int16_t xCom, yCom, zCom;
            compass_get(&xCom, &yCom, &zCom);
            // Add calibrated bias
            xCom += xComOff;
            yCom += yComOff;
            // calculate heading
            float compassHeading;
            compassHeading = atan2(yCom, xCom) - compassOffset; // returns -pi, pi
            // Update predicted state:    
            float error = (compassHeading - predictedTheta);
            vFunc_Inf2pi(&error);
            
            //kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
            ///* Commented back in due to fixed encoder
            if (fabs(error) > (0.8727*period_in_S)) { // 0.8727 rad/s is top speed while turning
                // If we have a reading over this, we can safely ignore the compass
                // Ignore compass while driving in a straight line
                kalmanGain = 0;
                //led_clear(LED_YELLOW);
            } else if ((robot_is_turning == FALSE) && (dRobot == 0)) {
                // Updated (a posteriori) state estimate
                kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
                //led_set(LED_YELLOW);
            } else {
                kalmanGain = 0;
                //led_clear(LED_YELLOW);
            }
           
            predictedTheta += kalmanGain*(error);
			vFunc_Inf2pi(&predictedTheta);            
            
            // Updated (a posteriori) estimate covariance
            covariance_filter_predicted = (1 - kalmanGain) * covariance_filter_predicted;  

            // Write the predicted pose to the global queue
            PredictedPose.theta = predictedTheta;
            PredictedPose.x = predictedX;
            PredictedPose.y = predictedY;
            xQueueOverwrite(globalPoseQ, &PredictedPose);
            
            // Notify the pose controller about the updated position estimate
            xTaskNotifyGive(xPoseCtrlTask);

        } else {
            // Not connected, getting heading and gyro bias
            uint16_t i;
            uint16_t samples = 100;
            float gyro = 0;
            for (i = 0; i<=samples; i++) {
                gyro += gyro_get_dps_z();
            }
            
            int16_t xCom, yCom, zCom;
            compass_get(&xCom, &yCom, &zCom);
            xCom += xComOff;
            yCom += yComOff;
            
            // Initialize pose to 0 and reset offset variables
            predictedX = 0;
            predictedY = 0;
            predictedTheta = 0;
            
            compassOffset = atan2(yCom, xCom);    
            gyroOffset = gyro / (float)i;               
        }
    } // While(1) end
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

		/*	  
	  gLeftWheelTicks = 0;
	  gRightWheelTicks = 0;
	  */
	  struct sWheelTicks WheelTicks = {0};
	  xQueueOverwrite(globalWheelTicksQ, &WheelTicks);

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
		struct sWheelTicks WheelTicks = {0};
		/*
		taskENTER_CRITICAL();
		leftWheelTicks = gLeftWheelTicks;
		rightWheelTicks = gRightWheelTicks;
		taskEXIT_CRITICAL();
		*/
		xQueueReceive(globalWheelTicksQ, &WheelTicks, 0);
		leftWheelTicks = WheelTicks.leftWheel;
		rightWheelTicks = WheelTicks.rightWheel;

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

  #ifdef DEBUG
  	debug("s", "Stack overflow!\n");
  #endif
  
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

  /* Init and start tracing */
  //vTraceEnable(TRC_START);

  /* Initialize RTOS utilities  */
  movementQ = xQueueCreate(2, sizeof(uint8_t)); // For sending movements to vMainMovementTask (used in compass task only)
  poseControllerQ = xQueueCreate(1, sizeof(struct sPolar)); // For setpoints to controller
  scanStatusQ = xQueueCreate(1, sizeof(uint8_t)); // For robot status
  globalWheelTicksQ = xQueueCreate(1, sizeof(struct sWheelTicks));
  globalPoseQ = xQueueCreate(1, sizeof(struct sPose)); // For storing and passing the global pose estimate
  priorityOrderQ = xQueueCreate(1, sizeof(struct sPolar)); // For sending priority orders received from the server
  
  //xPoseMutex = xSemaphoreCreateMutex(); // Global variables for robot pose. Only updated from estimator, accessed from many
  //xUartMutex = xSemaphoreCreateMutex(); // Protected printf with a mutex, may cause fragmented bytes if higher priority task want to print as well
  //xTickMutex = xSemaphoreCreateMutex(); // Global variable to hold robot tick values

  xCommandReadyBSem = xSemaphoreCreateBinary();
  xCollisionBSem = xSemaphoreCreateBinary();

  BaseType_t ret;
  xTaskCreate(vMainCommunicationTask, "Comm", 250, NULL, 3, NULL);  // Dependant on IO, sends instructions to other tasks
#ifndef COMPASS_CALIBRATE
  xTaskCreate(vMainPoseControllerTask, "PoseCon", 125, NULL, 1, &xPoseCtrlTask);// Dependant on estimator, sends instructions to movement task //2
  xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 125, NULL, 5, NULL); // Independent task,
  ret = xTaskCreate(vMainSensorTowerTask,"Tower", 125, NULL, 2, NULL); // Independent task, but use pose updates from estimator //1
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
  xTaskCreate(compassTask, "compasscal", 2000, NULL, 4, NULL); // Task used for compass calibration, dependant on communication and movement task
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