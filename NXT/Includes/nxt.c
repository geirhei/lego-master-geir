//************************************************************************/
// File:			nxt.c
// Author:			Kristian Lien, NTNU Spring 2017              
//
// Functions and tasks specific for the LEGO NXT robot
// 
/************************************************************************/

#include "nxt.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "nxt_avr.h"
#include "display.h"
#include "motor.h"
#include "io.h"
#include "nxt_motors.h"
#include "task.h"
#include "nxt_lcd.h"
#include "led.h"

#define FALSE 0
#define TRUE 1

void vTask1000Hz( void *pvParameters );
void vTask1Hz( void *pvParameters );
static void prvSetupHardware( void );

void nxt_init(void) {
  nxt_avr_init();
  display_init();
  nxt_motor_init();
  io_init();
  vMotor_init();
  prvSetupHardware();
  
  xTaskCreate(vTask1000Hz, "1000Hz", 75, NULL, 5, NULL);
  xTaskCreate(vTask1Hz, "1Hz", 75, NULL, 1, NULL);
  
  display_clear(1);
}

static void prvSetupHardware( void )
{
  /* When using the JTAG debugger the hardware is not always initialised to
  the correct default state.  This line just ensures that this does not
  cause all interrupts to be masked at the start. */
  AT91C_BASE_AIC->AIC_EOICR = 0;
  
  /* Most setup is performed by the low level init function called from the 
  startup asm file. */
  
  /* Enable the peripheral clock. */
  //AT91F_PIOA_CfgPMC();
}

// Task that communicates with the internal AVR every 1 ms, and processes the motors.
// Tests the rectangular button, and if it is pressed turns off the NXT.
void vTask1000Hz( void *pvParameters ) {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  while(1) {
	vTaskDelayUntil(&xLastWakeTime, xDelay);
	nxt_avr_1kHz_update();
	nxt_motor_1kHz_process();
	
	vMotorCountUpdate();
	
	if(buttons_get() & 0x8) {
      nxt_motor_set_speed(0, 0, 1);
      nxt_motor_set_speed(1, 0, 1);
      nxt_motor_set_speed(2, 0, 1);
	  nxt_lcd_power_down();
	  nxt_avr_power_down();
	}
  }
}

void vTask1Hz( void *pvParameters ) {
  const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  static int prev_dongle_status = 0;
  
  extern volatile uint8_t gHandshook, gPaused;
  extern SemaphoreHandle_t xCommandReadyBSem, xControllerBSem;
  
  while(1) {
	vTaskDelayUntil(&xLastWakeTime, xDelay);
  
	if(dongle_connected() != prev_dongle_status) {
	  prev_dongle_status = dongle_connected();
	  if(!dongle_connected()) {
		// We are not connected or lost connection, reset flags
		gHandshook = FALSE;
		gPaused = FALSE;
		xSemaphoreGive(xCommandReadyBSem); // Let uart parser reset if needed
	  }
	  xSemaphoreGive(xControllerBSem); // let the controller reset if needed
	}
  }
}