/************************************************************************/
// File:			defines.h
// Author:			Erlend Ese, NTNU Spring 2016
// Defines located in one file
//
// Note:
//  Defines regarding FreeRTOS are not here, check
//      - FreeRTOSConfig.h
//      - port.c in portable/GCC/ATmega1284
//  Not all registers are redefined 
//      - Timer/Counter Control registers
//      - Interrupt control registers
//      - ADCs
//
/************************************************************************/

#ifndef DEFINES_H_
#define DEFINES_H_
/************************************************************************/
/* PHYSICAL CONSTANTS - If the robot is changed these need to be changed 
 Some of these will be sent to server during the start-up-handshake   
 Wheel factor is the circumference divided by ticks per rotation      
 -> PI*81.6/200 = 1.28mm Length the robot travels per tick              */
#define WHEELBASE_MM             192.0 /* Length between wheel centers  */
#define ROBOT_TOTAL_WIDTH_MM     "210" /* From outer rim to outer rim   */
#define ROBOT_TOTAL_LENGTH_MM    "190" /* From front to aft, total      */
#define ROBOT_AXEL_OFFSET_MM     "39"  /* From center of square         */
#define SENSOR_TOWER_OFFSET_X_MM "30"  /* From center of square         */ 
#define SENSOR_TOWER_OFFSET_Y_MM "0"   /* From center of square         */
#define SENSOR_OFFSET_RADIUS_MM  "22"  /* From center of tower          */
#define ROBOT_DEADLINE_MS        "200" /* Interval between measurements */
#define SENSOR1_HEADING_DEG      "0"   /* Sensor angle relative to body */
#define SENSOR2_HEADING_DEG      "90"
#define SENSOR3_HEADING_DEG      "180"
#define SENSOR4_HEADING_DEG      "270" 

#define WHEEL_FACTOR_MM 1.25   /* Calculated, measured and calibrated   */

/************************************************************************/
/* Program settings                                                     */
#define PERIOD_MOTOR_MS         20
#define PERIOD_ESTIMATOR_MS     30
#define PERIOD_SENSORS_MS       200
#define moveStop                0
#define moveForward             1
#define moveBackward            2
#define moveClockwise           3
#define moveCounterClockwise    4
#define moveLeftForward         5
#define moveRightForward        6
#define moveLeftBackward        7
#define moveRightBackward       8
#define moveArrived             9
#define F_CPU                   16000000UL

/************************************************************************/
/* MICROCONTROLLER PIN CONSTANTS                                        */
/* Distance sensor                                                      */
#define distSensPort    PORTA   /* Sensors connected to port A          */
#define distSensReg     DDRA    /* Port A register used                 */
#define distSensLeft    PINA0   /* Forward sensor connected to PA0      */
#define distSensRear    PINA1   /* Backward sensor connected to PA1     */
#define distSensRight   PINA2   /* Right sensor connected to PA2        */
#define distSensFwd     PINA3   /* Left sensor connected to PA3          */
#define distSensPower   PINA4   /* Pin connected to transistor          */

/* Motor control                                                        */
#define motorPortRight      PORTC   /* Right Motor connected to port C  */
#define motorRegRight       DDRC    /* Port C register used             */
#define motorRightForward   PINC6   /* Right motor, pin 1               */
#define motorRightBackward  PINC7   /* Right motor, pin 2               */

#define motorPortLeft       PORTD   /* Port D register used             */
#define motorRegLeft        DDRD    /* Port D register used             */
#define motorLeftBackward   PIND6   /* Left motor, pin 1                */
#define motorLeftForward    PIND7   /* Left motor, pin 2                */

/* Motor encoder                                                        */
#define encoderReg      DDRD        /*   Port D register used           */
#define encoderPinLeft  PIND2       /*   Left encoder connected to PD2  */
#define encoderPinRight PIND3       /*   Right encoder connected to PD3 */
#define leftWheelCount  INT0_vect   /* Interrupt vector for PD2         */
#define rightWheelCount INT1_vect   /* Interrupt vector for PD3         */

/* Servo                                                                */
#define servoReg        DDRD        /* Port D register used             */
#define servoPin        PIND4       /* Servo connected to pin 4         */
#define servoOCR        OCR1B       /* Output compare register for PD4  */

/* LED                                                                  */
#define ledPORT		    PORTA   /*  LEDs connected to port D            */
#define ledReg          DDRA    /*  Port D register used                */
#define ledGREEN	    PINA5   /*  Green LED connected                 */
#define ledYELLOW	    PINA6   /*  Yellow LED connected                */
#define ledRED		    PINA7   /*  Green LED connected                 */

/* Communication                                                        */
#define USART_BAUDRATE 38400UL  /* If changed, also change in nRF51 code*/
#define nRF17 PINB0
#define nRF18 PIND5             /* Available GPIO pins from nRF51 Dongle*/
#define nRF19 PINB2             /* p 17, 18, 19 is connected through LC */
#define nRF51_status INT2_vect  /* PB2 has interrupt available          */
#define nRF20 PINB4             /* p20 is connected via voltage divider */

#define DDR_SPI DDRB            /* Data direction register for SPI      */
#define DD_MOSI PINB5           /* MOSI pin                             */
#define DD_SCK  PINB7           /* SCK pin                              */
#define DD_MISO PINB6           /* MISO pin                             */
#define DD_SS   PINB4           /* SPI Slave select pin, must be output */
#define IMU_SS  PINB1           /* Slave select pin for IMU             */

#define TWI_PORT    PORTC       /* Port for TWI                         */
#define TWI_REG     DDRC        /* Data register direction for TWI      */
#define SDA         PINC1       /* TWI data pin                         */
#define SCL         PINC0       /* TWI clock pin                        */




/************************************************************************/
/* COMMANDS                                                             */
#define UPDATE "{U,"
#define HANDSHAKE "{H,"
#define STATUS "{S,"
#define STATUS_CONFIRM "{S,CON}"
#define STATUS_PAUSE "{S,PAU}"
#define STATUS_UNPAUSE "{S,UNP}"
#define STATUS_IDLE "{S,IDL}"
#define TRUE 1
#define FALSE 0

/************************************************************************/
/* Macros                                                               */
#define nRFconnected !(PINB & (1<<nRF19))
#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI


#endif /* DEFINES_H_ */