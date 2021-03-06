#include <string.h>

#include "FreeRTOS.h"
#include "network.h"
#include "crc.h"
#include "io.h"
#include "cobs.h"

#define ADDRESS         2

void network_receive(uint8_t *frame, uint8_t len);

void (*receive_callbacks[10])(uint8_t, uint8_t*, uint16_t);

void network_init(void) {
  io_set_bluetooth_receive_callback(network_receive);
}

void network_set_callback(uint8_t protocol, void (*cb)(uint8_t, uint8_t*, uint16_t)) {
  if(protocol == PROTOCOL_ARQ || protocol == PROTOCOL_SIMPLE) receive_callbacks[protocol] = cb;
}
    
uint8_t network_send(uint8_t remote_address, uint8_t protocol, uint8_t *data, uint16_t len) {
  uint8_t *packet = pvPortMalloc(len+4);
  uint8_t *encoded_data = pvPortMalloc(len+6);
  if(packet == NULL || encoded_data == NULL) {
    vPortFree(packet);
    vPortFree(encoded_data);
    return 0;
  }
  packet[0] = remote_address;
  packet[1] = ADDRESS;
  packet[2] = protocol;
  memcpy(packet+3, data, len);
  packet[3+len] = calculate_crc(packet, 3+len);
  cobs_encode_result result = cobs_encode(encoded_data, len+5, packet, len+4);
  if(result.status != COBS_ENCODE_OK) {
    vPortFree(packet);
    vPortFree(encoded_data);
    return 0;
  }
  encoded_data[result.out_len] = 0x00;
  io_send_bluetooth(encoded_data, result.out_len+1);
  vPortFree(packet);
  vPortFree(encoded_data);
  return 1;
}

uint8_t network_get_address(void) {
  return ADDRESS;
}

// Receives a complete network frame ending with 0x00 and passes the data to the correct protocol
void network_receive(uint8_t *frame, uint8_t len) {
  uint8_t *decoded_data = pvPortMalloc(len);
  cobs_decode_result result = cobs_decode(decoded_data, len, frame, len-1);
  
  if(result.status != COBS_DECODE_OK) {
    vPortFree(decoded_data);
    return;
  }
  if(decoded_data[result.out_len-1] != calculate_crc(decoded_data, result.out_len-1) ) {
    vPortFree(decoded_data);
    return;
  }
  uint8_t receiver = decoded_data[0];
  uint8_t sender = decoded_data[1];
  uint8_t protocol = decoded_data[2];
  if(receiver != ADDRESS) {
    vPortFree(decoded_data);
    return;
  }
  receive_callbacks[protocol](sender, decoded_data+3, result.out_len-4);
  vPortFree(decoded_data);
}