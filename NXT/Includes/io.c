/*
* io.c
*
*  Created on: 21. feb. 2017
*      Author: krilien
*
*   Functions for controlling and communcating with the IO-microcontroller
*/

#include <string.h>
#include <stdlib.h>
#include "cobs.h"
#include "io.h"
#include "buffer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "display.h"
#include "semphr.h"
#include "crc.h"
#include "functions.h"
#include "hs.h"
#include "network.h"

#define BUFFER_SIZE 128

//Ditance sensors calibration
uint8_t voltage_to_cm[4][256]={
  {   
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,80,78,74,73,71,70,69,67,64,63,60,60,59,57,57,54,54,52,51,51,
    51,48,47,47,46,46,46,42,41,41,41,39,39,38,37,37,36,36,36,35,34,
    34,33,33,33,32,31,31,30,30,30,29,29,29,28,27,27,27,27,27,27,26,
    26,25,25,25,25,24,24,24,24,24,23,23,23,22,22,22,22,22,22,21,21,
    21,21,20,20,20,20,20,20,19,19,19,19,19,18,18,18,18,18,18,18,18,
    18,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,15,15,15,15,15,
    15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,14,13,13,13,
    13,13,13,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,12,
    12,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
    11,11,11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10},
  { 
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,80,79,79,78,74,74,70,70,67,66,63,63,62,60,60,58,58,
    58,53,53,53,51,51,51,47,45,45,45,44,44,42,41,41,39,39,39,38,37,
    37,37,37,37,35,33,33,33,33,32,31,31,31,30,30,30,29,28,28,28,27,
    27,27,27,27,26,26,26,25,25,25,24,24,24,24,23,23,23,22,22,22,22,
    22,21,21,21,21,21,21,20,20,20,19,19,19,19,19,19,19,18,18,18,18,
    18,18,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,15,15,15,15,
    15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,13,13,13,13,13,
    13,13,13,13,13,13,13,13,13,13,12,12,11,11,11,11,11,11,11,11,11,
    11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10},
  { 
    0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,80,79,79,78,78,78,73,70,70,66,63,63,62,62,58,57,55,55,54,51,
    51,50,50,49,48,46,46,45,45,45,44,42,42,41,41,39,38,38,38,37,36,
    36,35,35,35,34,33,33,32,32,32,31,30,30,30,30,30,29,29,29,29,29,
    29,27,26,26,26,26,26,25,25,25,24,24,24,24,23,23,23,23,23,22,22,
    22,22,21,21,21,21,21,21,20,20,20,20,20,19,19,19,19,19,18,18,18,
    18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,
    16,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,13,13,13,13,
    13,13,13,13,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,
    12,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,10},
  {
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,80,79,78,76,76,74,70,70,67,65,65,64,64,59,58,58,
    58,56,53,53,51,51,51,50,50,50,46,46,46,44,44,44,41,40,39,38,38,
    38,37,36,36,35,35,35,34,33,33,32,32,32,31,30,30,30,30,30,29,29,
    29,28,28,28,27,27,27,26,26,26,26,25,25,25,25,25,24,24,24,23,23,
    23,23,22,22,22,22,22,21,21,21,21,20,20,20,20,20,20,19,19,19,19,
    19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,16,16,
    16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,14,14,14,14,14,
    14,14,14,14,14,14,14,14,14,13,13,13,13,13,13,13,13,13,13,13,12,
    12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,
    11,11,11,11,11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,10,10,10,10}
};

struct from_io {
	uint8_t		dist[4];
	int16_t		gyro_x;
	int16_t		gyro_y;
	int16_t		gyro_z;
	int16_t		compass_x;
	int16_t		compass_y;
	int16_t		compass_z;
	uint8_t		dongle_status;
};

struct from_io io_values;

void io_task(void *pvParamters);
uint8_t io_send(uint8_t *data, uint8_t len, uint8_t should_wait);
uint8_t io_format_and_send(uint8_t *data, uint8_t len);
uint8_t io_send(uint8_t *data, uint8_t len, uint8_t should_wait);
void get_io_values(void);
void bluetooth_receive(void);
uint8_t io_should_wait(uint8_t type);
struct message_t io_message_unpack(uint8_t *msg, uint8_t len);
void (*bluetooth_callback)(uint8_t*, uint16_t);

struct message_t {
  uint8_t type;
  uint8_t len;
  char contents[BUFFER_SIZE];
  uint8_t crc;
};

typedef enum
{
  FREE		            = 0x00,
  WAITING               = 0x01
} io_status;

typedef enum
{
  SET_LED				= 0x01,
  SEND_BT				= 0x02,
  RECEIVE_BT			= 0x03,
  RECEIVE_SENSORS	    = 0x04,
  ALIVE_TEST            = 0x05
} nxt_message_type;

typedef enum
{
  SENSOR_DATA			= 0x01,
  BT_DATA				= 0x02,
  ALIVE_RESPONSE        = 0x03
} io_message_type;

buffer_t send_buffer;
buffer_t msg_type_buffer;

SemaphoreHandle_t send_queue_mutex;

uint8_t io_alive = 0;

void io_task(void *pvParamters) {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  char message[BUFFER_SIZE];
  uint8_t received_bytes[BUFFER_SIZE];
  uint8_t num_received_bytes = 0;
  cobs_decode_result cobs_result;
  
  uint16_t time = 0;
  uint8_t timeouts = 0;
  uint8_t number_of_errors = 0;
  uint8_t len = 0;
  uint16_t io_counter = 0;
  uint16_t bt_counter = 0;
  io_status status = FREE;
  struct message_t io_message;
  
  while(1) { // Loop until a connection with the IO-card is confirmed
    if(!io_alive) {
      uint8_t data[4] = {0};
      uint8_t num = hs_read_delimiter(data, 0, 4, 0x00);
      if(num != 0 && data[1] == ALIVE_RESPONSE) {
        io_alive = 1;
        break;
      }
      data[0] = 0x03; // COBS overhead byte
      data[1] = ALIVE_TEST; //(0x05)
      data[2] = 0x3F; // CRC for 0x05 (ALIVE_TEST id)
      data[3] = 0x00; // Delimiter
      hs_write(data, 0, 4);
    }
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
  }
  
  while(1) { // Main IO-loop
    vTaskDelayUntil(&xLastWakeTime, xDelay);

    if(++io_counter % 30  == 0) { // Tell IO-module to send sensors every 30 ms ( The estimator task runs every 30 ms, and should have updated values every time)
      get_io_values();
      io_counter = 0;
    }
    if(++bt_counter == 50) { // Tell IO-module to send bluetooth messages (if any are stored) every 50 ms
      bluetooth_receive();
      bt_counter = 0;
    }
    xSemaphoreTake(send_queue_mutex, portMAX_DELAY);
    len = send_buffer.len;
    xSemaphoreGive(send_queue_mutex);

    if(status == FREE && len > 0) { // If the rs485 line is available and send buffer is not empty: send the first message in the buffer
      xSemaphoreTake(send_queue_mutex, portMAX_DELAY);
      uint16_t num_bytes = buffer_remove_token(&send_buffer, message, 0x00, BUFFER_SIZE); // Read from the send buffer until the 0x00 is found (0x00 signals end of message to be sent)
      buffer_remove(&msg_type_buffer, &status, 1); // Get the status the IO should go to after sending this message (IDLE if no response to this message is expected)
      xSemaphoreGive(send_queue_mutex);

      hs_write(message, 0, num_bytes); // Send the bytes using the RS485 interface

      time = 0;
      
    } else if(status != FREE && time <= 20)  { // Increase the timer if we are waiting for a message from the IO module
      time += 1;
    } else if(status != FREE && time > 20){ //Have been waiting for a response for 20ms, assume message lost and make the bus available to send more data
      time = 0;
      status = FREE;
      display_goto_xy(0,0);
      display_int(++timeouts, 2);
      display_update();
    }
    
    num_received_bytes = hs_read_delimiter((unsigned char*) received_bytes, 0, BUFFER_SIZE, 0x00); // Get a message from the receive buffer if a complete message has been received (signaled by 0x00 at the end)
    
    if(num_received_bytes > 0) {

      cobs_result = cobs_decode(message, BUFFER_SIZE, received_bytes, num_received_bytes-1);
    
      if(cobs_result.status != COBS_DECODE_OK) continue; // COBS error

      io_message = io_message_unpack(message, cobs_result.out_len);

      if(calculate_crc(message, cobs_result.out_len-1) != io_message.crc) { // CRC error
        number_of_errors++;
        display_goto_xy(5,0);
        display_int(number_of_errors, 2);
        display_update();
        continue;
      } 
      if((io_message.type == SENSOR_DATA && status == WAITING) || (io_message.type == BT_DATA && status == WAITING)) { // Free the send bus when a message we were waiting for arrives
        status = FREE;
        time = 0;
      }
      if(io_message.type == SENSOR_DATA) {
        vTaskSuspendAll(); //Prevent another task from seeing inconsistent io data
        memcpy((void*) &io_values, io_message.contents, sizeof(io_values)); // Populate io values with the new data
        xTaskResumeAll();
        /*display_clear(0);
        display_goto_xy(0,2);
        display_int(voltage_to_cm[0][ io_values.dist[0] ], 3);
        display_int(voltage_to_cm[1][ io_values.dist[1] ], 3);
        display_int(voltage_to_cm[2][ io_values.dist[2] ], 3);
        display_int(voltage_to_cm[3][ io_values.dist[3] ], 3);
        display_update();*/

      } else if(io_message.type == BT_DATA && io_message.len > 0) {  //Does the message contain anything other than message ID and CRC? If not then no BT data was ready at the IO-micocontroller
        uint16_t i;
        uint16_t frame_start = 0;
        uint8_t *tmp = pvPortMalloc(MAX_FRAME_SIZE);
        static uint8_t buffer[MAX_FRAME_SIZE];
        static uint8_t buffer_count = 0;
        
        if(tmp == NULL) return;
        
        for(i=0;i<io_message.len;i++) {
          if(io_message.contents[i] == 0x00) { // Look for end of frame
            if(buffer_count != 0) { // Bytes belonging to this frame have been stored previously
              memcpy(tmp, buffer, buffer_count);
              memcpy(tmp+buffer_count, io_message.contents, i+1);
              bluetooth_callback(tmp, buffer_count+i+1);
              frame_start = i+1;
              buffer_count = 0;
            } else {
              memcpy(tmp, io_message.contents+frame_start, i-frame_start+1); 
              bluetooth_callback(tmp, i-frame_start+1);
              frame_start = i+1;
            }
          }
        }
        if(frame_start != io_message.len) { // More bytes remaining, belonging to another frame, store them to combine with the rest of the message when it arrives
          memcpy(buffer+buffer_count, io_message.contents+frame_start, io_message.len-frame_start);
          buffer_count += (io_message.len-frame_start);
        }
        vPortFree(tmp);
      }
    }
  }
}

void io_set_bluetooth_receive_callback(void (*cb)(uint8_t*, uint16_t)) {
  bluetooth_callback = cb;
}

uint8_t io_init(void) {
  hs_init();
  hs_enable(BAUD_RATE);
  char *buf = pvPortMalloc(500);
  char *type_buffer = pvPortMalloc(50);
  send_queue_mutex = xSemaphoreCreateMutex();
  
  if(buf == NULL || type_buffer == NULL) {
    display_goto_xy(0,0);
    display_string("IO init malloc error");
    display_update();
    return 0;
  }
  
  buffer_init(&send_buffer, buf, 500); // Buffer to store outgoing messages until they are sent by the io task.
  buffer_init(&msg_type_buffer, type_buffer, 50); // Buffer to store the types of the messages in the send buffer
  
  uint8_t init[5] = {0x01, 0x02, 0x03, 0x04, 0x00};
  hs_write(init, 0, 5); //First byte sent after boot is always wrong for some reason. Sending some dummy text to stop actual data from being corrupted  
  
  xTaskCreate(io_task, "IO", 1000, NULL, 5, NULL);
  
  return 1;
}

uint8_t io_send_bluetooth(uint8_t *data, uint16_t len) {
  uint8_t *message = pvPortMalloc(len+1);
  
  if(message == NULL) return 0;
  
  message[0] = SEND_BT;
  memcpy(message+1, data, len);
  
  io_format_and_send(message, len+1);
  
  vPortFree(message);
  
  return 1;
}

uint8_t io_send_bluetooth_string(char *str) {
  return io_send_bluetooth((uint8_t*) str, strlen(str));
}

//Add CRC, encode the message with COBS and add it to the send buffer
uint8_t io_format_and_send(uint8_t *data, uint8_t len) {
  uint8_t *message = pvPortMalloc(len+1);
  uint8_t *encoded_message = pvPortMalloc(len+3);
  uint8_t ret_value = 0;
  memcpy(message, data, len);
  message[len] = calculate_crc(message, len);
  cobs_encode_result result = cobs_encode(encoded_message, len+3, message, len+1);
  
  if(result.status != COBS_ENCODE_OK) {
    vPortFree(message);
    vPortFree(encoded_message);
    return 0;
  }
  encoded_message[result.out_len] = 0x00; //Add message delimiter
  
  ret_value = io_send(encoded_message, result.out_len+1, io_should_wait(message[0]));
  
  vPortFree(message);
  vPortFree(encoded_message);
  
  return ret_value;
}

uint8_t io_send_led_command(uint8_t leds) {
  uint8_t message[2] = {SET_LED, leds};
  return io_format_and_send(message, 2);
}

//Add the data to the send buffer
uint8_t io_send(uint8_t *data, uint8_t len, uint8_t should_wait) {
  if(data == NULL || len <= 0 || should_wait > 1 || !io_alive) return 0;
  
  io_status tmp = should_wait ? WAITING : FREE;
  
  uint8_t success = 1;
  
  xSemaphoreTake(send_queue_mutex, portMAX_DELAY);
  success *= buffer_append(&send_buffer, data, len);
  success *= buffer_append(&msg_type_buffer, &tmp, 1);
  xSemaphoreGive(send_queue_mutex);
  
  if(success == 0) {
    display_goto_xy(0,0);
    display_string("Full buffer!");
    display_update();
    return 0;
  }
  return 1;
}

void bluetooth_receive(void) {
  uint8_t encoded_message[4] = {0x03, RECEIVE_BT, 0xE2, 0x00}; //Raw data to be sent: RECEIVE_SENSORS (0x03). 0xE2 is the calculated crc for 0x03, 0x00 is the message delimiter, and the initial 0x03 is used for the cobs encoding
  io_send(encoded_message, 4, 1);
}

//Tell the IO-microcontroller to send io-values
void get_io_values(void) {
  uint8_t encoded_message[4] = {0x03, RECEIVE_SENSORS, 0x61, 0x00}; //Raw data to be sent: RECEIVE_SENSORS (0x04). 0x61 is the calculated crc for 0x04, 0x00 is the message delimiter, and the initial 0x03 is used for the cobs encoding
  io_send(encoded_message, 4, 1);
}

//Should we wait for a response to this message before sending anything more?
uint8_t io_should_wait(uint8_t type) {
  switch(type) {
    case RECEIVE_BT:
    case RECEIVE_SENSORS:
      return 1;
    default:
      return 0;
  }
}

struct message_t io_message_unpack(uint8_t *msg, uint8_t len) {
  struct message_t message;
  message.type = msg[0];
  message.len = len-2;
  memcpy(message.contents, msg+1, len-2);
  message.crc = msg[len-1];
  return message;
}


void gyro_get_raw(int16_t *xGyro, int16_t *yGyro, int16_t *zGyro) {
  *xGyro = io_values.gyro_x;
  *yGyro = io_values.gyro_y;
  *zGyro = io_values.gyro_z;
}

float gyro_get_dps_x(void) {
    return (float) io_values.gyro_x * 4.375 / 1000; //Calculate degrees per second from raw value
}
float gyro_get_dps_y(void) {
    return (float) io_values.gyro_y * 4.375 / 1000; //Calculate degrees per second from raw value
}
float gyro_get_dps_z(void) {
    return (float) io_values.gyro_z * 4.375 / 1000; //Calculate degrees per second from raw value
}

void compass_get(int16_t *xCom, int16_t *yCom, int16_t *zCom) {
  *xCom = io_values.compass_x;
  *yCom = io_values.compass_y;
  *zCom = io_values.compass_z;
}

uint8_t dongle_connected(void) {
  return io_values.dongle_status;
}

uint8_t distance_get_cm(uint8_t direction) {
  return voltage_to_cm[direction][ io_values.dist[direction] ];
}

uint8_t distance_get_volt(uint8_t direction) {
  return io_values.dist[direction];
}