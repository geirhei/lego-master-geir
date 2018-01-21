/************************************************************************/
// File:			main.c
// Author:			Erlend Ese, NTNU Spring 2016
//                  Modified for use with NXT by Kristian Lien, Spring 2017
//                  Modified by Geir Eikeland, Fall 2017
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
//#include "event_groups.h" /* RTOS event group related APT prototypes */

#include <stdlib.h>         // For itoa();
#include <string.h>         // For stringstuff
#include <stdio.h>
#include <math.h>

#include "display.h"
#include "hs.h"
#include "motor.h"
#include "server_communication.h"
#include "types.h"
#include "defines.h"
#include "functions.h"
#include "nxt.h"
#include "nxt_motors.h"
#include "led.h"
#include "io.h"
#include "arq.h"
#include "simple_protocol.h"
#include "network.h"
//#include "emlist.h"
#include "sensor_tower.h"
#include "pose_controller.h"
#include "navigation.h"
#include "mapping.h"

/* Semaphore handles */
SemaphoreHandle_t xCommandReadyBSem;
SemaphoreHandle_t xBeginMergeBSem;

/* Queues */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t globalWheelTicksQ = 0;
QueueHandle_t globalPoseQ = 0;
QueueHandle_t measurementQ = 0;

/* Task handles */
TaskHandle_t xPoseCtrlTask = NULL;
TaskHandle_t xMappingTask = NULL;

// Flag to indicate connection status.
volatile uint8_t gHandshook = FALSE;
volatile uint8_t gPaused = FALSE;
volatile message_t message_in;

/// Task declarations
void vMainCommunicationTask( void *pvParameters );
//void vMainSensorTowerTask( void *pvParameters );
//void vMainPoseControllerTask( void *pvParameters );
void vARQTask( void *pvParameters );
void vMainPoseEstimatorTask( void *pvParameters );
//void vMainMappingTask( void *pvParameters );
void vMainNavigationTask( void *pvParameters );

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );

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
	cartesian_t Target = {0}; // Structs for target coordinates from server
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
				case TYPE_ORDER:
					// Coordinates received in cm, convert to mm for internal use in the robot.
					Target.x = (float) command_in.message.order.x * 10;
					Target.y = (float) command_in.message.order.y * 10;
					/* Relay new coordinates to position controller */
					xQueueSendToFront(poseControllerQ, &Target, 0);
					break;
				case TYPE_PAUSE:
					//led_set(LED_YELLOW);
					// Stop sending update messages
					taskENTER_CRITICAL();
					gPaused = TRUE;
					taskEXIT_CRITICAL();
					// Stop controller - pass the current position
					xQueuePeek(globalPoseQ, &Target, 0);
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
					// Stop controller - pass the current position
					xQueuePeek(globalPoseQ, &Target, 0);
					xQueueOverwrite(poseControllerQ, &Target);
					break;
			}

		}
	}
}





/* Pose estimator task */
// New values and constants should be calibrated for the NXT
void vMainPoseEstimatorTask( void *pvParameters ) {
	//#define COMPASS_ENABLED

    int16_t previous_ticksLeft = 0;
    int16_t previous_ticksRight = 0;  
    
    const TickType_t xDelay = PERIOD_ESTIMATOR_MS;
    float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0f;
    
    float kalmanGain = 0.5;
    
    float predictedTheta = 0.0; // Start-heading i 90 degrees ?
    float predictedX = 0.0;
    float predictedY = 0.0;
    pose_t PredictedPose = {0};
    
    float gyroOffset = 0.0;
    float compassOffset = 0.0;
    
    // Found by using calibration task
    //int16_t xComOff = 11; 
    //int16_t yComOff = -78;
    int16_t xComOff = -321; 
    int16_t yComOff = -25;
    
    float variance_gyro = 0.0482f; // [rad] calculated offline, see report
    float variance_encoder = (2.0f * WHEEL_FACTOR_MM) / (WHEELBASE_MM); // approximation, 0.0257 [rad]
    
    float variance_gyro_encoder = (variance_gyro + variance_encoder) * period_in_S; // (Var gyro + var encoder) * timestep
    float covariance_filter_predicted = 0;
    
    #ifdef COMPASS_ENABLED
    #define CONST_VARIANCE_COMPASS 0.0349f // 2 degrees in rads, as specified in the data sheet
	#define COMPASS_FACTOR 10000.0f// We are driving inside with a lot of interference, compass needs to converge slowly
    #endif

    #ifndef COMPASS_ENABLED
    #define CONST_VARIANCE_COMPASS 0.0f
    #define COMPASS_FACTOR 0.0f
    #endif

    float gyroWeight = 0.5;//encoderError / (encoderError + gyroError);
    uint8_t robot_is_turning = 0;
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        // Loop
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );
        if (gHandshook) { // Check if we are ready    
            int16_t leftWheelTicks = 0;
            int16_t rightWheelTicks = 0;
            wheel_ticks_t WheelTicks = {0};

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
            #ifdef COMPASS_ENABLED
            // Get compass data: ( Request and recheck after 6 ms?)
            int16_t xCom, yCom, zCom;
            compass_get(&xCom, &yCom, &zCom);
            // Add calibrated bias
            xCom += xComOff;
            yCom += yComOff;
            // calculate heading
            float compassHeading = atan2(yCom, xCom) - compassOffset; // returns -pi, pi
            //debug("%f", compassHeading);

            // Update predicted state:    
            float error = (compassHeading - predictedTheta);
            //float error = 0; // Compass data not included
            vFunc_Inf2pi(&error);
            #endif
            #ifndef COMPASS_ENABLED
            float error = 0.0;
            #endif

            kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
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
			vFunc_Inf2pi(&predictedTheta); // Converts to (-pi,pi]
            
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
            gyroOffset = gyro / (float)i;

            // Initialize pose to 0 and reset offset variables (isn't this done at start of task?)
            /*
            predictedX = 0;
            predictedY = 0;
            predictedTheta = 0;
            */
            
            #ifdef COMPASS_ENABLED
            int16_t xCom, yCom, zCom;
            compass_get(&xCom, &yCom, &zCom);
            xCom += xComOff;
            yCom += yComOff;
            compassOffset = atan2(yCom, xCom);
            #endif
            
        }
    } // While(1) end
}



// For testing memory allocation
/*
void vMainNavigationTask( void *pvParameters )
{	
	float *distances;
	distances = pvPortMalloc(360 * sizeof(float));
	for (uint8_t i = 0; i < 360; i++) {
		distances[i] = INFINITY;
	}

	float headings[4];

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{	
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// Receive a measurement
		measurement_t Measurement = {0};
		xQueueReceive(measurementQ, &Measurement, 0);

		// Read the global pose
		pose_t Pose = {0};
		xQueuePeek(globalPoseQ, &Pose, 0);

		float robotHeading = Pose.theta;
		uint8_t servoStep = Measurement.servoStep;
		navigation_get_measurement_headings(robotHeading, servoStep, headings);

		/*
		LinkedList *frontierLocations = emlist_create();
		for (uint8_t i = 0; i < 1000; i++) {
			uint32_t a = 1;
			uint32_t *ptr = &a;
			emlist_insert(frontierLocations, ptr);
		}
		while(!emlist_is_empty(frontierLocations)) {
			emlist_pop(frontierLocations);
		}
		emlist_destroy(frontierLocations);
		*/
/*
	}
}
*/


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
	  
	  //uint8_t movement;
	  //movement = moveCounterClockwise;
	  //xQueueSendToBack(movementQ, &movement, 10);
	  uint8_t leftDirection = motorBackward;
	  uint8_t rightDirection = motorForward;
	  vMotorMovementSwitch(-25, 25, &leftDirection, &rightDirection);

	  float heading = 0;
	  float gyroHeading = 0;
	  float encoderHeading = 0;

		/*	  
	  gLeftWheelTicks = 0;
	  gRightWheelTicks = 0;
	  */
	  wheel_ticks_t WheelTicks = {0};
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
		/*
		taskENTER_CRITICAL();
		leftWheelTicks = gLeftWheelTicks;
		rightWheelTicks = gRightWheelTicks;
		taskEXIT_CRITICAL();
		*/
		wheel_ticks_t WheelTicks = {0};
		if (xQueueReceive(globalWheelTicksQ, &WheelTicks, 0) == pdTRUE) {
			leftWheelTicks = WheelTicks.leftWheel;
			rightWheelTicks = WheelTicks.rightWheel;
		}

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
	  //movement = moveClockwise;
	  leftDirection = motorForward;
	  rightDirection = motorBackward;
	  vMotorMovementSwitch(25, -25, &leftDirection, &rightDirection);
	  //xQueueSendToBack(movementQ, &movement, 10);
	  //movement = moveStop;
	  //xQueueSendToBack(movementQ, &movement, 10);
	  //vMotorMovementSwitch(0, 0, &leftDirection, &rightDirection);
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

  // assert test
  //uint8_t a = 0;
  //configASSERT(a == 1);

  /* Init and start tracing */
  //vTraceEnable(TRC_START);
  
  /* Initialize RTOS utilities  */
  movementQ = xQueueCreate(2, sizeof(uint8_t)); // For sending movements to vMainMovementTask (used in compass task only)
  poseControllerQ = xQueueCreate(1, sizeof(cartesian_t)); // For setpoints to controller
  scanStatusQ = xQueueCreate(1, sizeof(uint8_t)); // For robot status
  globalWheelTicksQ = xQueueCreate(1, sizeof(wheel_ticks_t));
  globalPoseQ = xQueueCreate(1, sizeof(pose_t)); // For storing and passing the global pose estimate
  measurementQ = xQueueCreate(1, sizeof(measurement_t));

  xCommandReadyBSem = xSemaphoreCreateBinary();
  xBeginMergeBSem = xSemaphoreCreateBinary();

  // For debugging
  vQueueAddToRegistry(movementQ, "Movement queue");
  vQueueAddToRegistry(poseControllerQ, "Pose controller queue");
  vQueueAddToRegistry(scanStatusQ, "Scan status queue");
  vQueueAddToRegistry(globalWheelTicksQ, "Global wheel ticks queue");
  vQueueAddToRegistry(globalPoseQ, "Global pose queue");
  vQueueAddToRegistry(measurementQ, "Measurement queue");

  vQueueAddToRegistry(xCommandReadyBSem, "Command ready semaphore");
  vQueueAddToRegistry(xBeginMergeBSem, "Begin merge semaphore");

  BaseType_t ret;
  xTaskCreate(vMainCommunicationTask, "Comm", 250, NULL, 3, NULL);  // Dependant on IO, sends instructions to other tasks
#ifndef COMPASS_CALIBRATE
  xTaskCreate(vMainPoseControllerTask, "PoseCon", 125, NULL, 1, &xPoseCtrlTask);// Dependant on estimator, sends instructions to movement task //2
  xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 125, NULL, 5, NULL); // Independent task,
  //xTaskCreate(vMainMappingTask, "Mapping", 500, NULL, 5, NULL);
  //xTaskCreate(vMainNavigationTask, "Navigation", 500, NULL, 5, NULL);
  ret = xTaskCreate(vMainSensorTowerTask,"Tower", 125, NULL, 2, &xMappingTask); // Independent task, but use pose updates from estimator //1
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