#include "pose_estimator.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <math.h>

#include "defines.h"
#include "types.h"
#include "functions.h"
#include "io.h"

extern volatile uint8_t gHandshook;

extern QueueHandle_t globalPoseQ;
extern QueueHandle_t globalWheelTicksQ;
extern TaskHandle_t xPoseCtrlTask;

void vMainPoseEstimatorTask( void *pvParameters )
{
    /* Task init */
    const TickType_t xDelay = PERIOD_ESTIMATOR_MS;
    float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0f;
    
    float kalmanGain = 0.5;
    
    float predictedTheta = 0.0; // Start-heading i 90 degrees ?
    float predictedX = 0.0;
    float predictedY = 0.0;
    pose_t PredictedPose = {0};
    
    float gyroOffset = 0.0;
    //float compassOffset = 0.0;
    
    // Found by using calibration task
    //int16_t xComOff = 11; 
    //int16_t yComOff = -78;
    //int16_t xComOff = -321; 
    //int16_t yComOff = -25;
    
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

    wheel_ticks_t WheelTicks = { 0, 0 };
    wheel_ticks_t PreviousWheelTicks = WheelTicks;
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Loop
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );
        if (gHandshook) { // Check if we are ready    
            
            // Set initial pose change values to zero                
            float dRobot = 0;
            float dTheta = 0;

            // Read wheel ticks from queue written to by motor driver
            // This part is executed only if there is a new tick-value in the queue
            if (xQueueReceive(globalWheelTicksQ, &WheelTicks, 0) == pdTRUE) {

                // Distance wheels have travelled since last sample
                float dLeft = (float) (WheelTicks.left - PreviousWheelTicks.left) * WHEEL_FACTOR_MM; 
                float dRight = (float) (WheelTicks.right - PreviousWheelTicks.right) * WHEEL_FACTOR_MM;
            
                PreviousWheelTicks.left = WheelTicks.left;
                PreviousWheelTicks.right = WheelTicks.right;
                       
                dRobot = (dLeft + dRight) / 2;
                // Get angle from encoders, dervied from arch of circles formula
                dTheta = (dRight - dLeft) / WHEELBASE_MM;
            }
            
            
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
            #endif /* COMPASS_ENABLED */

            #ifndef COMPASS_ENABLED
            float error = 0.0;
            #endif /* COMPASS_ENABLED */

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