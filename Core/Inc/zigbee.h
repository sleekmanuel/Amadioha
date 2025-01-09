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


#define XBEE_SUCCESS        0
#define XBEE_ERROR_RESPONSE 1
#define XBEE_ERROR_TIMEOUT  2

// Zigbee Device Addresses
#define ZIGBEE_COORDINATOR_ADDRESS    0x0000  // Address for the coordinator
#define ZIGBEE_ROUTER_ADDRESS         0x1234  // Example address for a router
#define ZIGBEE_END_DEVICE_ADDRESS     0x5678  // Example address for an end device

// Broadcast Address
#define ZIGBEE_BROADCAST_ADDRESS      0xFFFF

// Maximum Transmission Unit (MTU) Buffer Sizes
#define ZIGBEE_TX_BUFFER_SIZE         128     // Transmission buffer size
#define ZIGBEE_RX_BUFFER_SIZE         128     // Reception buffer size

// Retry Parameters
#define ZIGBEE_MAX_RETRIES            3       // Maximum number of retries
#define ZIGBEE_RETRY_DELAY_MS         200     // Delay between retries in milliseconds

// Timeout Configuration
#define ZIGBEE_ACK_TIMEOUT_MS         500     // Acknowledgment timeout in milliseconds

// Other Zigbee Parameters
#define ZIGBEE_DEFAULT_CHANNEL        17      // Default channel
#define ZIGBEE_PAN_ID                 0x1221  // Personal Area Network ID

void enterCommandMode(void);
void setDestinationAddress(uint32_t DH, uint32_t DL);
void writeCommand(void);
int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length);
void FactoryReset();
void XBee_NodeDiscovery(void);
void ATNI(void);  //temp node identifier function
void exitCommandMode(void);

#endif /* INC_ZIGBEE_H_ */
