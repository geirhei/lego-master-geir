/************************************************************************/
// File:			functions.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Various functions needed
//
/************************************************************************/

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <stdint.h>

#include "types.h"

/* Take any angle and put it inside -pi,pi */
void vFunc_Inf2pi(float *angle_in_radians);

/* Wrap any angle in radians into the interval [0,2pi) */
void vFunc_wrapTo2Pi(float *angle_in_radians);

/* Parse the update message from uart by using tokens */
void vFunc_ParseUpdate(char *cin, float *theta, int16_t *radius);

// reverses a string 'str' of length 'len'
void vFunc_reverse(char* p, char* q);

// increase function for decimal parsing
char* vFunc_inc(char* s, char* e);

// C program for implementation of ftoa()
char* vFunc_ftoa(float num, char* dest, int afterPoint);

void itoa(int n, char s[]);
void reverse(char s[]);

point_t vFunc_polar2Cart(float theta, float r);

float func_get_slope(line_t *Line);

float func_distance_between(point_t *Pos1, point_t *Pos2);

int8_t vFunc_isMergeable(line_t *Line1, line_t *Line2);

point_t func_get_projected_point(point_t P, float a, float b);

#endif /* FUNCTIONS_H_ */
