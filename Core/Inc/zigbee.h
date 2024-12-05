/*
 * zigbee.h
 *
 *  Created on: Dec 2, 2024
 *      Author: root
 */

#ifndef INC_ZIGBEE_H_
#define INC_ZIGBEE_H_

#include "main.h"
#include <stdio.h>
#include <string.h>

#define Data_BUFFER_SIZE 12
#define XBEE_SUCCESS        0
#define XBEE_ERROR_RESPONSE 1
#define XBEE_ERROR_TIMEOUT  2



void setDestinationAddress(uint32_t DH, uint32_t DL);
void writeCommand(void);
int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length);

#endif /* INC_ZIGBEE_H_ */
