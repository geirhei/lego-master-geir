/**
 *	Author: Geir Eikeland
 *	Written as part of a master's thesis at NTNU, fall 2017
 *
 * Functions used by the navigation task
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
//#include "semphr.h"

//#include <stdint.h>

//#include "types.h"

void vMainNavigationTask( void *pvParameters );

//void navigation_get_measurement_headings(float robotHeading, uint8_t servoStep, float *headings);

#endif