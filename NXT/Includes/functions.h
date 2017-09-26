/************************************************************************/
// File:			functions.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Various functions needed
//
/************************************************************************/

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <stdint.h>

/* Take any angle and put it inside -pi,pi */
void vFunc_Inf2pi(float *angle_in_radians);

/* Parse the update message from uart by using tokens */
void vFunc_ParseUpdate(char *cin, float *theta, int16_t *radius);

// reverses a string 'str' of length 'len'
void vFunc_reverse(char* p, char* q);

// increase function for decimal parsing
char* vFunc_inc(char* s, char* e);

// C program for implementation of ftoa()
char* vFunc_ftoa(double num, char* dest, int afterPoint);

void itoa(int n, char s[]);
void reverse(char s[]);

#endif /* FUNCTIONS_H_ */
