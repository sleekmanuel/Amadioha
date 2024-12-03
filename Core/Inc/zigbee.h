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

void requestSerialNumberLow(void);
void requestDestNumberLow(void);
void setDestinationAddress(uint32_t DH, uint32_t DL);
void writeCommand(void);

#endif /* INC_ZIGBEE_H_ */
