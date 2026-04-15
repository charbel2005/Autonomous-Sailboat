#ifndef LORA_H
#define LORA_H

#include <stdint.h>



int LoRa_init(void);
void LoRa_Send(uint8_t *data, uint8_t len);
int  LoRa_Receive(uint8_t *buf, uint32_t timeout_ms);


#endif


