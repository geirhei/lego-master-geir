/************************************************************************/
// File:			pose_estimator.h
// Author:			- Erlend Ese, NTNU Spring 2016
// 					- Position estimation improved and kalman filter
// 					algorithm added by Jørund Amsen, NTNU spring 2017
// 					- Moved to seperate file and made some structural
// 					changes by Geir Eikeland, NTNU spring 2018
// 
// Contains the function which implements the task that estimates the
// pose of the robot. The algorithm is currently using only the ticks
// from the wheel encoder and the rotational data from the gyro in the
// estimation process. The code for the compass has been commented out
// for now, and should not be enabled before exact calibrations have been
// made.
//
/************************************************************************/

#ifndef POSE_ESTIMATOR_H_
#define POSE_ESTIMATOR_H_

/**
 * @brief      The main pose estimator task containing the kalman filter
 *             algorithm for calculating the current position and heading.
 *
 * @param      pvParameters  The pv parameters
 */
void vMainPoseEstimatorTask( void *pvParameters );

#endif