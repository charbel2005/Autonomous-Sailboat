#ifndef LORA_H
#define LORA_H

#include <stdint.h>



int LoRa_init(void);
void LoRa_Send(uint8_t *data, uint8_t len);


#endif


