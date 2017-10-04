/************************************************************************/
// File:			servo.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Servo driver for sensor tower
/************************************************************************/

/*  AVR includes    */
#include <avr/io.h>

/*  Custom includes    */
#include "servo.h"
#include "defines.h"    
    
/************************************************************************/
//     Values found by testing and then linearized in matlab
//     0 degrees = 2180, 90 degrees = 3615
//     Basic linear algebra yields:
//      y = 15.94*x + 2180
// MATLAB CODE START
// n = 90;
// for i=1:n+1
//     x(i) = 15.94 * (i-1) + 2180;
//     array = transpose(round(x));
// end
// disp('Check variable array for row-table of pwm values');
// MATLAB CODE STOP
// Array index contains corresponding calibrated PWM value.
/************************************************************************/

/************************************************************************/
// Array to map angle from degrees to pulse-width in order to avoid
// floating point operation. More details in declaration below
/************************************************************************/
    const uint16_t  DEG_TO_PWM[91] = {
        1750, 1763, 1776, 1788, 1801, 1814, 1827, 1839, 1852, 1865, 1878, 
        1891, 1903, 1916, 1929, 1942, 1954, 1967, 1980, 1993, 2006, 2018, 
        2031, 2044, 2057, 2070, 2082, 2095, 2108, 2121, 2133, 2146, 2159, 
        2172, 2185, 2197, 2210, 2223, 2236, 2248, 2261, 2274, 2287, 2300, 
        2312, 2325, 2338, 2351, 2363, 2376, 2389, 2402, 2415, 2427, 2440, 
        2453, 2466, 2478, 2491, 2504, 2517, 2530, 2542, 2555, 2568, 2581, 
        2593, 2606, 2619, 2632, 2645, 2657, 2670, 2683, 2696, 2709, 2721, 
        2734, 2747, 2760, 2772, 2785, 2798, 2811, 2824, 2836, 2849, 2862, 
        2875, 2887, 2900
    };

/************************************************************************/
// Initializes PWM for correct pins and timer for the servo
// Non-inverted, fast PWM, clear on compare match, 20ms period, presc 8
// Output: PortD Pin 4
/************************************************************************/
void vServo_init(uint8_t servoAngleDeg){
    /* Clear OCnA/OCnB on Compare Match, set */
    /* OCnA/OCnB at BOTTOM (non-inverting mode) */
    /* Datasheet p.132 Table 14-3 */
    TCCR1A |= (1<<COM1B1) | (0<<COM0B0);
    
    /* Waveform generation mode 14: Fast PWM */
    /* top: ICRn, Update bottom, flag set on top */
    /* Datasheet p.133 Table 14-5 */
    TCCR1B |= (1<<WGM13) | (1<<WGM12);
    TCCR1A |= (1<<WGM11) | (0<<WGM10);

    /* Clock select bit description: */
    /* clkI/O/8 (From prescaler) - Datasheet p.134 Table 14-6*/
    TCCR1B |= (0<<CS12) | (1<<CS11) | (0<<CS10);
    
    /* 50Hz 20ms period => 20Mhz/(8clk*50Hz) - 1 = ICR1] */
    /* Datasheet p.125 */
    ICR1 = 39999; // 49999 for 20mhz, 39 999 for 16mhz
    
    /*PortD Pin 4 as servo PWM Output (OC1B)*/
    servoReg |= (1<<servoPin);
    
    /*  Set angle to desired start angle (usually 0)*/
    vServo_setAngle(servoAngleDeg);
}

/* Sets servo angle to a specific degree */
void vServo_setAngle(uint8_t ServoAngleDeg){
    /* Ensure feasible values */
    if (ServoAngleDeg >= 90){
        ServoAngleDeg = 90;
    }
    else if(ServoAngleDeg <= 0){
        ServoAngleDeg = 0;
    }
    /* Fetch pulse width from array and set to output */
    servoOCR = DEG_TO_PWM[ServoAngleDeg];
}