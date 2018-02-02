/************************************************************************/
// File:			functions.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Various functions needed
//
/************************************************************************/

#include "functions.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>


/* Take any angle and put it inside -pi,pi */
void vFunc_Inf2pi(float *angle_in_radians){
    do{
        if (*angle_in_radians > M_PI) *angle_in_radians -= 2*M_PI;
        else if (*angle_in_radians < -M_PI) *angle_in_radians += 2*M_PI;
    } while (fabs(*angle_in_radians) > M_PI);
}

/* Wrap any angle in radians into the interval [0,2pi) */
void func_wrap_to_2pi(float *angle_in_radians) {
    do {
        if (*angle_in_radians >= 2*M_PI) *angle_in_radians -= 2*M_PI;
        else if (*angle_in_radians < 0) *angle_in_radians += 2*M_PI;
    } while (fabs(*angle_in_radians) >= 2*M_PI);
}

/* Parse the update message from uart by using tokens */
void vFunc_ParseUpdate(char *cin, float *theta, signed short *radius){
    char *token;
    const char delimiters[] = "{U,}\n"; // U for update
    token = strtok(cin, delimiters);
    unsigned char i = 0;
    while (token != NULL){
        switch (i)
        {
            case 0:
                *theta = atoi(token);
            break;
            
            case 1:{
                *radius = atoi(token);
            break;}
            
            default:
            break;
        }
        token = strtok(NULL, delimiters);
        i++;
    }
}

/* reverses a string 'str' of length 'len' */
void vFunc_reverse(char* p, char* q)
{
    char c;
    for(; p < q; ++p, --q){
        c = *p;
        *p = *q;
        *q = c;
    }
}

/* increase function for decimal parsing */
char* vFunc_inc(char* s, char* e)
{
    int co = 1;
    char* t = e;
    
    //increase from end to start
    for(; t >= s; --t){
        if(*t == '.') continue;
        *t += co;
        if(*t > '9'){
            *t = '0';
            co = 1;
        }
        else{
            co = 0;
            break;
        }
    }
    //check if there's still carry out
    if(co){
        for(t = ++e; t > s; --t) *t = *(t - 1);
        *s = '1';
    }
    return e;
}

/* C program for implementation of ftoa() */
char* vFunc_ftoa(float num, char* dest, int afterPoint){
    char* p = dest;
    int integer = (int)num;
    float decimal = num - integer;
    
    //parse sign
    if(num < 0){
        *p++ = '-';
        integer = -integer;
        decimal = -decimal;
    } else {
		*p++ = ' ';
	}
    //parse integer
    if(integer){
        char* q = p;
        for(; integer; integer /= 10){
            *q++ = '0' + integer % 10;
        }
        vFunc_reverse(p, q - 1);
        p = q;
    }
    else *p++ ='0';
    //parse decimal
    if(afterPoint > 0){
        *p++ ='.';
        for(; afterPoint; --afterPoint){
            decimal *= 10;
            *p++ = '0' + (int)decimal;
            decimal -= (int)decimal;
        }
        if((int)(decimal + 0.5)) p = vFunc_inc(dest, p - 1) + 1;
    }
    
    *p = '\0';
    return dest;
}

void itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}

void reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

point_t func_polar2cart(float theta, float r) {
    float x = r * cos(theta);
    float y = r * sin(theta);
    return (point_t) { x, y };
}

float func_get_slope(line_t *Line) {
    return (Line->Q.y - Line->P.y) / (Line->Q.x - Line->P.x);
}

float func_distance_between(point_t *Pos1, point_t *Pos2) {
    return sqrt( pow(Pos1->x - Pos2->x, 2) + pow(Pos1->y - Pos2->y, 2) );
}

point_t func_get_projected_point(point_t P, float a, float b) {
    float x = (P.x + a * P.y - a * b) / (1 + pow(a, 2));
    float y = (a * P.x + pow(a, 2) * P.y + b) / (1 + pow(a, 2));
    return (point_t) { x, y };
}
