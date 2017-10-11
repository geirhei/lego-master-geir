/*  Sensor tower task */
void vMainSensorTowerTask( void *pvParameters){
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
  
  while(1){
	// Loop
	if ((gHandshook == TRUE) && (gPaused == FALSE)){
	  // xLastWakeTime variable with the current time.
	  xLastWakeTime = xTaskGetTickCount();
	  // Set scanning resoltuion depending on which movement the robot is executing.
	  // Note that the iterations are skipped while robot is rotating (see further downbelow)
	  if (xQueueReceive(scanStatusQ, &robotMovement,150 / portTICK_PERIOD_MS) == pdTRUE){
        
		//printf("\n\tNew movement: %i\n",robotMovement);
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
	  vMotorSetAngle(servoTower, servoStep*servoResolution);
	  
	  // Wait total of 200 ms for servo to reach set point and allow previous update message to be transfered.
	  vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_PERIOD_MS );   
	  
	  // Get measurements from sensors
      uint8_t forwardSensor = distance_get_cm(0);
      uint8_t leftSensor = distance_get_cm(1);
      uint8_t rearSensor = distance_get_cm(2);
      uint8_t rightSensor = distance_get_cm(3);
      
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
	  send_update(ROUND(xhat/10),ROUND(yhat/10),ROUND(thetahat*RAD2DEG),servoStep*servoResolution, forwardSensor, leftSensor, rearSensor, rightSensor);
	  
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
	}
	else{ // Disconnected or unconfirmed
	  vMotorSetAngle(servoTower, 0);
	  // Reset servo incrementation
	  rotationDirection = moveCounterClockwise;
	  servoStep = 0;
	  vTaskDelay(100/portTICK_PERIOD_MS);
	}
  }// While end
}

/**
 * @brief      Task for controlling the movement of the robot. Depends on
 *             semaphore from estimator task.
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
  uint8_t USE_GYRO_COMPASS = 0;
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
  const int16_t xComOff = -278; 
  const int16_t yComOff = 13;
  
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
	  taskENTER_CRITICAL();
	  current_leftWheelTicks = gLeftWheelTicks;
	  current_rightWheelTicks = gRightWheelTicks;
	  taskEXIT_CRITICAL(); 
	  float dLeft = (float)(current_leftWheelTicks - previous_leftWheelTicks) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
	  float dRight = (float)(current_rightWheelTicks - previous_rightWheelTicks) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
	  previous_leftWheelTicks = current_leftWheelTicks;
	  previous_rightWheelTicks = current_rightWheelTicks;
	  
	  float dRobot = (dLeft + dRight) / 2; // Distance robot has travelled since last sample
	  float dTheta = (dRight - dLeft) / WHEELBASE_MM; // [RAD] Get angle from encoders, derived from arch of circles formula
	  
	  /* PREDICT */
	  // Get gyro data:
	  float gyrZ = DEG2RAD * (gyro_get_dps_z() - gyroOffset);
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
	  if(USE_GYRO_COMPASS) predictedTheta += fusedHeading;
	  else predictedTheta += dTheta;
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
	  vFunc_Inf2pi(&compassHeading);
	  
	  
	  if (xQueueReceive(driveStatusQ, &driveStatus, 0) == pdTRUE){
		// Check if we start driving to a new location in a straight line
		if (driveStatus == moveForward){
		  //led_set(LED_RED);
		  driveStatus = moveStop;
		  oldCompass = compassHeading;
		}
		
		// Check if we have arrived after driving in a straight line
		else if (driveStatus == moveArrived){
		  //led_clear(LED_RED);
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
		//led_set(LED_YELLOW);
	  }
	  else{
		// Dont update compass while robot is driving, to avoid false curved trajectory
		kalmanGain = 0.0;
		//led_clear(LED_YELLOW);
	  }      
	  
	  // Update predicted state:
	  float error;
	  error = (compassHeading - predictedTheta);
	  vFunc_Inf2pi(&error);
	  
	  if(USE_GYRO_COMPASS) predictedTheta  += kalmanGain*(error);
	  vFunc_Inf2pi(&predictedTheta); 
	  
	  
	  // Updated (a posteriori) estimate covariance
	  covariance_filter_predicted = (1 - kalmanGain) * covariance_filter_predicted;  
	  
	  // Update pose
	  xSemaphoreTake(xPoseMutex,20);
	  gTheta_hat = predictedTheta;//theeta;
	  gX_hat = ROUND(predictedX);
	  gY_hat = ROUND(predictedY);
	  xSemaphoreGive(xPoseMutex);
	  /*char str[10];
	  vFunc_ftoa(gTheta_hat, str, 5);
	  display_clear(0);
	  display_goto_xy(0,3);
	  display_int(gX_hat, 5);
	  display_int(gY_hat, 5);
	  display_goto_xy(0,4);
	  display_string(str);
	  display_update();*/
	  // Send semaphore to controller
	  xSemaphoreGive(xControllerBSem);
	}
	else{
	  // Not connected, getting heading and gyro bias
	  uint16_t i;
	  uint16_t samples = 500;
	  float gyro = 0;
	  for (i = 0; i <= (samples-1); i++){
		gyro += gyro_get_dps_z();
	  }
	  
	  int16_t xCom = 0;
	  int16_t yCom = 0;
	  int16_t zCom = 0;

	  compass_get(&xCom, &yCom, &zCom);       
	  
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