/************************************************************************/
// File:			sensor_tower.h
// Author:			- Erlend Ese, NTNU Spring 2016
// 					- Moved into separate file and adjusted tower behaviour by Geir
//			        Eikeland, NTNU Spring 2018.
//
// Contains the main task responsible for control of the sensor tower.
// Interaction with the mapping task has been added to the code for passing
// distance measuremeents through a queue. Uses direct-to-task notification to
// the mapping task.
//
// /************************************************************************/

#ifndef SENSOR_TOWER_H_
#define SENSOR_TOWER_H_

/**
 * @brief      The main task responsible for control of the sensor tower and
 *             some low-level collision handling.
 *
 * @param      pvParameters  The pv parameters
 */
void vMainSensorTowerTask( void *pvParameters );

#endif