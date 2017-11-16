/**
 *	Author: Geir Eikeland
 *	Written as part of a master's thesis at NTNU, fall 2017
 *
 * Functions used by the navigation task
 */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include <stdint.h>

/**
 * @brief      Calculates the headings of the measurements relative to the global world.
 *
 * @param[in]  robotHeading  The robot heading
 * @param[in]  servoStep     The number of increments the IR-tower has turned clockwise at the time of measurement
 * @param      headings      The array to fill with the calculated heading values
 */
void navigation_get_measurement_headings(float robotHeading, uint8_t servoStep, float *headings);

#endif