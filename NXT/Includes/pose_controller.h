/************************************************************************/
// File:			pose_controller.h
// Author:			- Erlend Ese, NTNU Spring 2016
// 					- Moved into separate file and added new synchronisation
// 					mechanisms by Geir Eikeland, NTNU Spring 2018.
//
// Contains the main task responsible for control of the servos for the wheels.
// It receives setpoints from the communication task, and uses a PI-controller
// algorithm to regulate the motor actuation.
//
// /************************************************************************/

#ifndef POSE_CONTROLLER_H_
#define POSE_CONTROLLER_H_

/**
 * @brief      The main task that is responsible for control of the wheels.
 *             Takes in setpoints in (x,y)-coordinates and uses a PI-controller
 *             to adjust the actuation from readings of the global pose.
 *
 * @param      pvParameters  The pv parameters
 */
void vMainPoseControllerTask( void *pvParameters );

#endif