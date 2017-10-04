/************************************************************************/
// File:			main.c
// Author:			Erlend Ese, NTNU Spring 2016
//                  Credit is given where credit is due.
// Purpose:
// AVR Robot with FreeRTOS implementation in the collaborating robots
// project.
// In FreeRTOS each thread of execution is known as tasks
// and are written as C functions
//
// MCU: ATmega1284p, programmed with JTAG ICEMK2
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
// GLOBAL VARIABLES:
// See line 86
//
// HARDWARE SETUP
//  Servo pin:  Port D pin 4 (18)
//  Sensor pins: Port A pin 0 - 4 (36-40)
//  Motor pins: Port D pin 6, 7 & Port C pin 6, 7 (20,21,28,29)
//  Encoder pins: Port D pin 1, 2 (14, 15)
//
// TIMERS USED:
//  Timer3 FreeRTOS (tick increments)
//  Timer1 Servo (PWM)
//
// Interrupt routines located after main function
/************************************************************************/

/* KERNEL INCLUDES */
#include "FreeRTOS.h" /* Must come first */
#include "task.h"     /* RTOS task related API prototypes */
#include "semphr.h"   /* Semaphore related API prototypes */
#include "queue.h"    /* RTOS queue related API prototypes */

/* AVR INCLUDES    */
#include <stdlib.h>         // For itoa();
#include <string.h>         // For stringstuff
#include <util/atomic.h>    // For atomic operation
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

/* Semaphore handles */
SemaphoreHandle_t xScanLock;
SemaphoreHandle_t xPoseMutex;
SemaphoreHandle_t xControllerBSem;
SemaphoreHandle_t xCommandReadyBSem;

/* Queues */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t actuationQ = 0;
QueueHandle_t driveStatusQ = 0;

/* CUSTOM LIBRARIES    */
#include "defines.h"
#include "LED.h"
#include "servo.h"
#include "motor.h"
#include "distSens_g2d12.h"
#include "usart.h"
#include "spi.h"
#include "twi_master.h"
#include "imu_LSM6DS3.h"
#include "com_HMC5883L.h"
#include "functions.h"
#include "arq.h"
#include "server_communication.h"
#include "simple_protocol.h"
#include "network.h"
#include "bat.h"
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

message_t message_in;

// Might need this for debuging func
volatile uint8_t gDebug = FALSE;

// Global robot pose
float gTheta_hat = 0;
int16_t gX_hat = 0;
int16_t gY_hat = 0;

/* STRUCTURE */
struct sPolar{
    float heading;
    int16_t distance;
};

#ifdef DEBUG
#warning DEBUG IS ACTIVE

#endif

//#define tictoc
#ifdef tictoc
    // Pin for timing tasks, use tic/toc - PINB3 is free
    #define usetictoc DDRB |= (1<<PINB3)
    #define tic PORTB |= (1<<PINB3)
    #define toc PORTB &= ~(1<<PINB3)
#endif

/*  Communication task */
void vMainCommunicationTask( void *pvParameters ){
	// Setup for the communication task
	struct sPolar Setpoint = {0}; // Struct for setpoints from server

	message_t command_in; // Buffer for recieved messages

	server_communication_init();
	xTaskCreate(vARQTask, "ARQ", 500, NULL, 3, NULL);
	uint8_t success = 0;
	
	while(!success) {
		success = server_connect();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		vLED_toggle(ledGREEN);
	}
	
	send_handshake();
	
	while(1){
		if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY) == pdTRUE){
			// We have a new command from the server, copy it to the memory
			vTaskSuspendAll ();       // Temporarily disable context switching
			taskENTER_CRITICAL();
			command_in = message_in;
			taskEXIT_CRITICAL();
			xTaskResumeAll ();      // Enable context switching
			switch(command_in.type){
				case TYPE_CONFIRM:
					taskENTER_CRITICAL();
					gHandshook = TRUE; // Set start flag true
					taskEXIT_CRITICAL();

					break;
				case TYPE_PING:
					send_ping_response();
				break;
				case TYPE_ORDER:
					Setpoint.heading = command_in.message.order.orientation;
					Setpoint.distance = command_in.message.order.distance;
					// Ensure max values are not exceeded
					if (Setpoint.distance > 320){
						Setpoint.distance = 320;
					}
					else if (Setpoint.distance < -320){
						Setpoint.distance = -320;
					}
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
			// Command is processed
		} // if (xCommandReady) end
	}// While(1) end
}// vMainComtask end

/*  Sensor tower task */
void vMainSensorTowerTask( void *pvParameters){
    /* Task init */
    #ifdef DEBUG
        debug"Tower OK\n");
    #endif 
        
    float thetahat = 0;
    int16_t xhat = 0;
    int16_t yhat = 0;
    uint8_t count = 0;
	
    uint8_t rotationDirection = moveCounterClockwise;
    uint8_t servoStep = 0;
    uint8_t servoResolution = 1;
    uint8_t robotMovement = moveStop;
    
    uint8_t idleCounter = 0;
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    
    while(1){
        // Loop
        if ((gHandshook == TRUE) && (gPaused == FALSE)){
            // xLastWakeTime variable with the current time.
            xLastWakeTime = xTaskGetTickCount();
            // Set scanning resoltuion depending on which movement the robot is executing.
            // Note that the iterations are skipped while robot is rotating (see further downbelow)
            if (xQueueReceive(scanStatusQ, &robotMovement,150 / portTICK_PERIOD_MS) == pdTRUE){
                //debug("\n\tNew movement: %i\n",robotMovement);
                switch (robotMovement)
                {
                case moveStop:
                    servoStep *= servoResolution;
                    servoResolution = 1;
                    idleCounter = 1;
                    break;
                case moveForward:
                case moveBackward:
                    servoResolution = 6;
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
            
            vServo_setAngle(servoStep*servoResolution);
  
            // Wait total of 200 ms for servo to reach set point and allow previous update message to be transfered.
            vTaskDelayUntil(&xLastWakeTime, 140 / portTICK_PERIOD_MS );   
            vDistSens_On();
            // Let sensor voltage stabilize like described in datasheet.
            vTaskDelay(60 / portTICK_PERIOD_MS);  
            // Get measurements from sensors
            uint8_t forwardSensor = ui8DistSens_readCM(distSensFwd);
            uint8_t leftSensor = ui8DistSens_readCM(distSensLeft);
            uint8_t rearSensor = ui8DistSens_readCM(distSensRear);
            uint8_t rightSensor = ui8DistSens_readCM(distSensRight);
            
            vDistSens_Off();
            
            // Get latest pose estimate
            xSemaphoreTake(xPoseMutex,40);
                thetahat = gTheta_hat;
                xhat = gX_hat;
                yhat = gY_hat;
            xSemaphoreGive(xPoseMutex);
            
            if ((idleCounter > 10) && (robotMovement == moveStop)){
                // If the robot stands idle for 1 second, send 'status:idle' in case the server missed it.
                send_idle();
                idleCounter = 1;
            }
            else if ((idleCounter >= 1) && (robotMovement == moveStop)){
                idleCounter++;
            }
            
            //Send updates to server in the correct format (centimeter and degrees)
            send_update(xhat/10,yhat/10,thetahat*RAD2DEG,servoStep*servoResolution,forwardSensor,leftSensor,rearSensor,rightSensor);
            
			
			count ++;
			// Send bat.update
			if (count == 30){
				send_battery(adc_readBat());
				count = 0;
			}
		
			
            // Low level anti collision
            uint8_t objectX;
            
            if ((servoStep*servoResolution) <= 30) objectX = forwardSensor;// * cos(servoStep*5);
            else if((servoStep*servoResolution) >= 60) objectX = rightSensor;// * cos(270 + servoStep*5);
            else objectX = 0;
            
            if ((objectX > 0) && (objectX < 2)){
                // Stop controller
                struct sPolar Setpoint = {0, 0};
                xQueueSend(poseControllerQ, &Setpoint, 100);
            }            
            
            
            // Iterate in a increasing/decreasing manner and depending on the robots movement
            if ((servoStep*servoResolution <= 90) && (rotationDirection == moveCounterClockwise) && (robotMovement < moveClockwise)){
                servoStep++;                
            } 
            else if ((servoStep*servoResolution > 0) && (rotationDirection == moveClockwise) && (robotMovement < moveClockwise)){
                servoStep --;
            }
            
            if ((servoStep*servoResolution >= 90) && (rotationDirection == moveCounterClockwise)){
                rotationDirection = moveClockwise;
            }
            else if ((servoStep*servoResolution <= 0) && (rotationDirection == moveClockwise)){
                rotationDirection = moveCounterClockwise;
            }          
        }
        else if (gPaused == TRUE && gHandshook == TRUE){
            vTaskDelay(200 / portTICK_PERIOD_MS);
            /* Sensor calibration */
            /*
            vServo_setAngle(0);
            vDistSens_On();
            vTaskDelay(50 / portTICK_PERIOD_MS);
            uint8_t forwardSensor = ui8DistSens_readAnalog(distSensFwd);
            uint8_t leftSensor = ui8DistSens_readAnalog(distSensLeft);
            uint8_t rearSensor = ui8DistSens_readAnalog(distSensRear);
            uint8_t rightSensor = ui8DistSens_readAnalog(distSensRight);
            vDistSens_Off();
            debug("%i \t %i \t %i \t %i\n",forwardSensor,leftSensor,rearSensor,rightSensor);
            vTaskDelay(150 / portTICK_PERIOD_MS);
            */
        }
        else{ // Disconnected or unconfirmed
            vServo_setAngle(0);
            // Reset servo incrementation
            rotationDirection = moveCounterClockwise;
            servoStep = 0;
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }// While end
}

/*  Calculates new settings for the movement task */
void vMainPoseControllerTask( void *pvParameters ){
    #ifdef DEBUG
        debug("PoseController OK\n");
    #endif
    /* Task init */    
    struct sPolar Setpoint = {0}; // Struct for updates
    struct sPolar Error = {0}; // Struct for error
    struct sPolar Epsilon = {0.1152,1};
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
            if (xSemaphoreTake(xControllerBSem, portMAX_DELAY) == pdTRUE){    // Wait for synchronization from estimator
                // Get robot pose
                xSemaphoreTake(xPoseMutex,portMAX_DELAY);
                    thetahat = gTheta_hat;
                    xhat = gX_hat;
                    yhat = gY_hat;
                xSemaphoreGive(xPoseMutex);
                       
                // Check if a new update is recieved
                if (xQueueReceive(poseControllerQ, &Setpoint, 0) == pdTRUE){
                    xQueueReceive(poseControllerQ, &Setpoint, 10); // Recieve theta and radius setpoints from com task, wait for 10ms if nessecary
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
                        int16_t xSquaredCm = xhat * xhat / 100;
                        int16_t ySquaredCm = yhat * yhat / 100;
                        referenceModel.distance = sqrt(xSquaredCm + ySquaredCm);
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
                        
                        float pidOutput = (80*Error.heading + integrator - dHeading);
                        
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
                    int16_t diffX = (xInit - xhat) / 10;
                    int16_t diffY = (yInit - yhat) / 10;
                    
                    diffX *= diffX;
                    diffY *= diffY;
                    referenceModel.distance = referenceModel.distance + (abs(Setpoint.distance) - referenceModel.distance) / 10;
                    Error.distance = referenceModel.distance - sqrt((diffX + diffY));
                             
                    // Since we use cutoff to stop the robot we need to check the condition in a separate variable without the reference model.
                    int16_t errorDistance = abs(Setpoint.distance) - sqrt((diffX + diffY));
                    
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
                        int16_t dXY = (sqrt((diffX + diffY)) - oldVal.distance) / 0.030;
                        
                        integrator += Error.distance;
                        if (integrator >= 100) integrator = 100;
                        
                        
                        int16_t pidOutput = 50*Error.distance + (int16_t)integrator - dXY;
                        
                        if (pidOutput > 100) pidOutput = 100;
                        else if (pidOutput < 0) pidOutput = 0;
                        
                        uint8_t actuation = (uint8_t)pidOutput;
                        
                        xQueueSend(movementQ, &moveDir, 0);
                        xQueueSend(actuationQ, &actuation, 0); 
                        oldVal.distance = sqrt((diffX + diffY));
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

/* Pose estimator task */
void vMainPoseEstimatorTask( void *pvParameters ){
    
    uint8_t driveStatus = 0;
    uint8_t compass_is_reliable = FALSE;
    
    int16_t previous_leftWheelTicks = 0;
    int16_t previous_rightWheelTicks = 0;  
    
    double kalmanGain = 0;
    double covariance_filter_predicted = 0;
    
    float predictedTheta = 0.0;
    float oldCompass = 0.0;

    float predictedX = 0.0;
    float predictedY = 0.0;
    
    float gyroOffset = 0.0;
    float compassOffset = 0.0;
 
    // These may need to be updated when exploring a new area
    // Can be found by using the calibration task and using
    // define COMPASS_CALIBRATE
    const int16_t xComOff = 103; 
    const int16_t yComOff = -35;
    
    const float variance_gyro = 0.0482f; // [rad] calculated offline, see report
    const float variance_encoder = (2.0f * WHEEL_FACTOR_MM) / (WHEELBASE_MM); // approximation, 0.0257 [rad]
    const TickType_t xDelay = PERIOD_ESTIMATOR_MS;
    const float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0f;
    const float variance_gyro_encoder = (variance_gyro + variance_encoder) * period_in_S; // (Var gyro + var encoder) * timestep
    
    
    
    #define CONST_VARIANCE_COMPASS 0.0349f // 2 degrees in rads, as specified in the data sheet
    #define COMPASS_FACTOR 10000.0f// We are driving inside with a lot of interference, compass needs to converge slowly
    
    float gyroWeight = 0;
    //float adaptiveGyroWeight = 0;
    
    uint8_t robot_is_turning = 0;
    
    #ifdef DEBUG
        debug("Estimator OK");
        debug("[%i]",PERIOD_ESTIMATOR_MS);
        debug("ms\n");   
    #endif
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    while(1){
        // Loop
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );   
        if (gHandshook){ // Check if we are ready     
            int16_t current_leftWheelTicks = 0;
            int16_t current_rightWheelTicks = 0;
            // Get encoder data, protect the global tick variables
            ATOMIC_BLOCK(ATOMIC_FORCEON){
                current_leftWheelTicks = gLeftWheelTicks;
                current_rightWheelTicks = gRightWheelTicks;
             }    
            float dLeft = (float)(current_leftWheelTicks - previous_leftWheelTicks) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
            float dRight =(float)(current_rightWheelTicks - previous_rightWheelTicks) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
            previous_leftWheelTicks = current_leftWheelTicks;
            previous_rightWheelTicks = current_rightWheelTicks;
                       
            float dRobot = (dLeft + dRight) / 2; // Distance robot has travelled since last sample
            float dTheta = (dRight - dLeft) / WHEELBASE_MM; // [RAD] Get angle from encoders, derived from arch of circles formula
            
            
            /* PREDICT */
            // Get gyro data:
            float gyrZ = DEG2RAD * (fIMU_readFloatGyroZ() - gyroOffset);
            // If the robot is not really rotating we don't include the gyro measurements, to avoid the trouble with drift while driving in a straight line
            if(fabs(gyrZ) < DEG2RAD*10){ 
                gyroWeight = 0; // Disregard gyro while driving in a straight line
                robot_is_turning = FALSE; // Don't update angle estimates
            }
            else {
                robot_is_turning = TRUE;                
                /*
                // Use the calculated relationship between encoder and gyro based on compass measurements *EXPERIMENTAL*
                if (adaptiveGyroWeight < 1 && adaptiveGyroWeight > 0){
                    gyroWeight = adaptiveGyroWeight;
                }
                else */
                // Gyro tends to show too small values, while encoders tends to show too large values.
                gyroWeight = 0.75; // Found by comparing encoders to gyro in the motion lab - this is dependent on the surface the robot is driving on!
            }      
            
            gyrZ *= period_in_S; // Scale gyro measurement
            
            // Fuse heading from sensors to predict heading:
            float fusedHeading = 0;
            if (robot_is_turning == TRUE){
             fusedHeading = (1 - gyroWeight) * dTheta + gyroWeight * gyrZ;  
            }
            else {
                fusedHeading = 0;
            }            
            
            // Estimate global X and Y pos
            predictedX = predictedX + (dRobot * cos(predictedTheta + 0.5 * fusedHeading));
            predictedY = predictedY + (dRobot * sin(predictedTheta + 0.5 * fusedHeading));

            // Predicted (a priori) state estimate for theta
            predictedTheta += fusedHeading;
                  
            // Predicted (a priori) estimate covariance
            covariance_filter_predicted += variance_gyro_encoder;

             /* UPDATE */
             // Get compass data:
             int16_t xCom, yCom, zCom;
             vCOM_getData(&xCom, &yCom, &zCom);
             // Add calibrated bias
             xCom += xComOff;
             yCom += yComOff;
             // calculate heading
             float compassHeading;
             compassHeading = atan2(xCom, yCom) - compassOffset ; // returns -pi, pi
             vFunc_Inf2pi(&compassHeading);
             
            
            if (xQueueReceive(driveStatusQ, &driveStatus, 0) == pdTRUE){
                xQueueReceive(actuationQ, &driveStatus, 10);
                // Check if we start driving to a new location in a straight line
                if (driveStatus == moveForward){
                    vLED_singleHigh(ledRED);
                    driveStatus = moveStop;
                    oldCompass = compassHeading;
                }
                
                // Check if we have arrived after driving in a straight line
                else if (driveStatus == moveArrived){
                    vLED_singleLow(ledRED);
                    driveStatus = moveStop;
                    float diffCompass = (compassHeading - oldCompass);
                    vFunc_Inf2pi(&diffCompass);
                    // Check if the compass has changed after driving in a straight line
                    if (fabs(diffCompass) < (CONST_VARIANCE_COMPASS)){
                        compass_is_reliable = TRUE;
                        /*
                        // Calculate relationship between encoder and gyro based on compass measurements *EXPERIMENTAL*
                        float numerator = (fabs(compassHeading) - fabs(dTheta));
                        vFunc_Inf2pi(&numerator);
                        float denominator = (fabs(gyrZ) - fabs(dTheta));
                        vFunc_Inf2pi(&denominator);
                        adaptiveGyroWeight = numerator / denominator;
                        */
                    }
                    else
                        compass_is_reliable = FALSE;
                        
                }                    
            }
            
            // Check if robot is not moving and that the compass is reliable            
            if ((robot_is_turning == FALSE) && (dRobot == 0) && (compass_is_reliable == TRUE)){
                // Updated (a posteriori) state estimate, use compass factor instead of variance to avoid too fast convergence toward inaccurate readings
                kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + COMPASS_FACTOR);
                //vLED_singleHigh(ledYELLOW);
            }
            else{
                // Dont update compass while robot is driving, to avoid false curved trajectory
                kalmanGain = 0.0;
                //vLED_singleLow(ledYELLOW);
            }      
               
            // Update predicted state:
            float error;
            error = (compassHeading - predictedTheta);
            vFunc_Inf2pi(&error);
            
            predictedTheta  += kalmanGain*(error);
            vFunc_Inf2pi(&predictedTheta); 
            
            
            // Updated (a posteriori) estimate covariance
            covariance_filter_predicted = (1 - kalmanGain) * covariance_filter_predicted;  
            
            // Update pose
            xSemaphoreTake(xPoseMutex,20);
                gTheta_hat = predictedTheta;
                gX_hat = predictedX;
                gY_hat = predictedY;
            xSemaphoreGive(xPoseMutex);
            // Send semaphore to controller
            xSemaphoreGive(xControllerBSem);
        }
        else{
            // Not connected, getting heading and gyro bias
            uint16_t i;
            uint16_t samples = 500;
            float gyro = 0;
            for (i = 0; i <= (samples-1); i++){
                gyro+= fIMU_readFloatGyroZ();
            }
            
            int16_t xCom = 0;
            int16_t yCom = 0;
            int16_t zCom = 0;

            vCOM_getData(&xCom, &yCom, &zCom);                         
            xCom += xComOff;
            yCom += yComOff; 
            
            // Initialize pose to 0 and reset offset variables
            predictedX = 0;
            predictedY = 0;
            predictedTheta = 0;

            compassOffset = atan2(xCom,yCom);
            oldCompass = -compassOffset;    
            gyroOffset = gyro / (float)i;
            vTaskDelay(200 / portTICK_PERIOD_MS);           
        }
		// Sends debug message
			if (gDebug)
			{
				debug("%d %d %f\n", gX_hat, gY_hat, gTheta_hat*RAD2DEG);	
			}
    } // While(1) end
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
    #ifdef DEBUG
    debug("Movement OK\n");
    #endif
    
    while(1){
        if (gHandshook){ // Check if we are connected and good to go
            xQueueReceive(movementQ, &movement, 0);
            xQueueReceive(actuationQ, &actuation, 0);
            if (movement != lastMovement){
                ATOMIC_BLOCK(ATOMIC_FORCEON){
                    bias_LeftWheelTick = gLeftWheelTicks;
                    bias_RightWheelTick = gRightWheelTicks;
                }
                lastMovement = movement;
                xQueueSend(scanStatusQ, &lastMovement, 0);
            }
            int16_t tmp_leftWheelTicks = 0;
            int16_t tmp_rightWheelTicks = 0;
            ATOMIC_BLOCK(ATOMIC_FORCEON){
                tmp_leftWheelTicks = gLeftWheelTicks - bias_LeftWheelTick;
                tmp_rightWheelTicks = gRightWheelTicks - bias_RightWheelTick;
            }
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
                    vMotorBrakeLeft();
                    vMotorBrakeRight();
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
            vMotorBrakeLeft();
            vMotorBrakeRight();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }





    }// While(1) end
}

/*#define COMPASS_CALIBRATE*/

#ifdef COMPASS_CALIBRATE
void compassTask(void *par){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int16_t xComOff = 0;
    int16_t yComOff = 0;
    while(1){
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    if (gHandshook){
        int16_t xComMax = -4000, yComMax = -4000;
        int16_t xComMin = 4000, yComMin = 4000;
        int16_t xCom, yCom, zCom;   
// wait until you start moving     
//         while(fabs(zGyr) < 20){
//             zGyr = fIMU_readFloatGyroZ();
//             
//             vTaskDelay(15/portTICK_PERIOD_MS);
//         }
        debug("Rotating in\n...3\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        debug("...2\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        debug("...1\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
            ATOMIC_BLOCK(ATOMIC_FORCEON){
                leftWheelTicks = gLeftWheelTicks;
                rightWheelTicks = gRightWheelTicks;
            }
            float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
            float dRight =(float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
            previous_ticksLeft = leftWheelTicks;
            previous_ticksRight = rightWheelTicks;
            float dTheta = RAD2DEG * (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
            
            float zGyr = 0.05*(fIMU_readFloatGyroZ() + 1.5490); // add offset bias
// Storing values for printing            
//             gyroHeading += zGyr;
//             encoderHeading += dTheta;
//             tabellE[tellar] = dTheta;
//             tabellG[tellar] = zGyr;
//             tellar++;
            heading += 0.5 * zGyr  + 0.5 * dTheta; // Complementary filter

                       
            vCOM_getData(&xCom, &yCom, &zCom);
            
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
//             debug("%.1f, %.1f\n",tabellG[i], tabellE[i]);
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//         }
//         debug("gyro %.2f, encoder %.2f \n", gyroHeading, encoderHeading);
        
// Printing new xy cal values
        debug("Old XY-offset values: %d %d\n", xComOff, yComOff);
        xComOff = ((xComMax - xComMin)/2) - xComMax;
        yComOff = ((yComMax - yComMin)/2) - yComMax;
        
        debug("New XY-offset values: %d %d\n", xComOff, yComOff);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    else
    vTaskDelay(200 / portTICK_PERIOD_MS); 
        
        
    }
}
#endif

/*  In case of stack overflow, disable all interrupts and handle it  */
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName){
    cli();
    /*  Handle overflow */
    #ifdef DEBUG
       debug("Overflow\n");
    #endif
    while(1){
        vLED_toggle(ledGREEN);
        //vLED_toggle(ledYELLOW);
        vLED_toggle(ledRED);
    }// While(1) end
}

/*  Main function   */
int main(void){
    /* Setup - Initialize all settings before tasks  */
    /* Initialize LED, pins defined in LED.h   */
    vLED_init();
    // Set red LED on to indicate INIT is ongoing
    vLED_singleHigh(ledRED); 
    
    /* Initialize USART driver, NB! baud is dependent on nRF51 dongle       */
    /* nRF51 dongle interrupt pin is set up in vMotor_interruptInit() below */
    vUSART_init();
    
	network_init();
	arq_init();
	simple_p_init(server_receiver);
	
    /* If the MCU resets, the reason can be seen in MCUSR register */
    // See page 56 in the data sheet
    #ifdef DEBUG
		uint8_t reg = MCUSR;
        debug("Reboot.\nStatus register: 0b%d\n", reg);

        MCUSR = 0; // Reset MCUSR
    #endif
    
    /* Initialize servo for sensor tower to zero degrees */
    vServo_init(0);
    
    /* Initialize sensors */
    vDistSens_init();
    
    /* Initialize motor controller */
    vMotor_init();
    
    /* Initialize interrupts */
    vMotor_interruptInit();

    /* Initialize Inertial Measurement Unit (IMU) and SPI */
    #ifdef DEBUG
        debug("IMU init..\n");
    #endif

    sIMU_begin();
    
    /* Initialize compass */
    /* Connected with I2C, if the chip has no power, MCU will lock.*/
    #ifdef DEBUG
        debug("Compass init..\n");
    #endif
    vCOM_init();

    /* Initialize RTOS utilities  */
    movementQ = xQueueCreate(1,sizeof(uint8_t));					// For sending movements to vMainMovementTask
    poseControllerQ = xQueueCreate(1, sizeof(struct sPolar));		// For setpoints to controller
    scanStatusQ = xQueueCreate(1,sizeof(uint8_t));					// For robot status
    actuationQ = xQueueCreate(1,sizeof(uint8_t));					// To send variable actuations to motor task
    driveStatusQ = xQueueCreate(1,sizeof(uint8_t));					// To send if robot is driving to the estimator
    
    xPoseMutex = xSemaphoreCreateMutex();							// Global variables for robot pose. Only updated from estimator, accessed from many
    	
    
    xControllerBSem = xSemaphoreCreateBinary();						// Estimator to Controller synchronization
    xCommandReadyBSem = xSemaphoreCreateBinary();					// uart ISR to comm task sync
    
    xTaskCreate(vMainMovementTask, "Movement", 500, NULL, 4, NULL);     // Independent task, uses ticks from ISR
    xTaskCreate(vMainCommunicationTask, "Comm", 1000, NULL, 3, NULL);    // Dependant on ISR from UART, sends instructions to other tasks
    
    #ifndef COMPASS_CALIBRATE
        
        xTaskCreate(vMainPoseControllerTask, "PoseCon", 500, NULL, 2, NULL);// Dependant on estimator, sends instructions to movement task
        xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 500, NULL, 5, NULL); // Independent task, uses ticks from ISR
        xTaskCreate(vMainSensorTowerTask,"Tower",500, NULL, 1, NULL);       // Independent task, but use pose updates from estimator
    #endif


    #ifdef COMPASS_CALIBRATE
        debug("\n \t WARNING \t !\n");
        debug("COMPASS CALIBRATION!\n");
        debug("Connect to begin\n");
        xTaskCreate(compassTask, "compasscal", 8000, NULL, 4, NULL); // Task used for compass calibration, dependant on communication and movement task
    #endif
        
    sei();
    
    
    #ifdef DEBUG
        debug("Starting scheduler ....\n");
    #endif
    
    #ifdef tictoc
        usetictoc;
    #endif
	vLED_singleLow(ledRED); 
    /*  Start scheduler */
    vTaskStartScheduler();
    
    /*  MCU is out of RAM if the program comes here */
    while(1){
        cli();
        debug("RAM fail\n");
        vLED_toggle(ledGREEN);
        vLED_toggle(ledRED);
        //vLED_toggle(ledYELLOW);
    }
}


/*
 Interrupt Service Routines
 Int0: Left wheel optical encoder
 Int1: Right wheel optical encoder
 Int2: nRF dongle status pin, NB set up in motor.c
 USART0 RX vector: USART with nRF dongle
*/

/* Handle tick from left wheel encoder, INT0 */
ISR(leftWheelCount){
    vMotorEncoderLeftTickFromISR(gLeftWheelDirection);
}

/* Handle tick from right wheel encoder, INT1 */
ISR(rightWheelCount){
    vMotorEncoderRightTickFromISR(gRightWheelDirection);
}

/* Handle change of connection status, INT2 */
ISR(nRF51_status){
    /*if (nRFconnected){
        // indicate we are connected
       // vLED_singleHigh(ledGREEN);
       // vLED_singleHigh(ledYELLOW);
    }
    else{
        // We are not connected or lost connection, reset flags
        gHandshook = FALSE;
        gPaused = FALSE;
       
        xSemaphoreGiveFromISR(xCommandReadyBSem,0); // Let uart parser reset if needed
    }
    xSemaphoreGiveFromISR(xControllerBSem,0); // let the controller reset if needed*/
}

