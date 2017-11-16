#include "navigation.h"

#include <stdint.h>

#include "defines.h"

void navigation_get_measurement_headings(float robotHeading, uint8_t servoStep, float *headings) {
	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		uint8_t heading = (int) robotHeading + servoStep + (i-1)*90;
		if (heading >= 360) {
			heading -= 360;
		} else if (heading < 0) {
			heading +=360;
		}
		headings[i] = heading;
	}
}