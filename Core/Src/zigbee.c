/*
 * zigbee.c
 *
 *  Created on: Dec 2, 2024
 *      Author: root
 */



#include <zigbee.h>

void enterCommandMode(void)
{
    char command_mode[3] = "+++";
    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    // Receive the "OK" response from XBee
    HAL_UART_Receive_IT(&huart1, (uint8_t*)XBeeData.rx_buffer, 3);
}


/*
 * Enter AT command mode, request parameter and exit AT command mode
 * @param at_command: at command to enter
 * @param output_buffer: xbee response for at command
 * @param length: length of output buffer
 */
int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length) {
    // Clear buffer and reset flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;

    char command_mode[] = "+++";
    char exit_command[] = "ATCN\r";

    // Enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t *)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    while (!XBeeData.data_received_flag);
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0) return XBEE_ERROR_RESPONSE;

    // Send the parameter request command
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    while (!XBeeData.data_received_flag);
    if (strlen((char *)XBeeData.rx_buffer) < length) return XBEE_ERROR_RESPONSE;
    memcpy(output_buffer, XBeeData.rx_buffer, length);

    // Exit AT command mode
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    while (!XBeeData.data_received_flag);
    return strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0 ? XBEE_SUCCESS : XBEE_ERROR_RESPONSE;
}



void writeCommand(){
    // Clear rx_buffer and reset the data_received_flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;

    char command_mode[3] = "+++";  //Command to enter AT command mode
    char at_command[] = "ATWR\r";  // Command to write to XBEE EEPROM

    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    HAL_UART_Receive_IT(&huart1,  &XBeeData.received_byte, 1);

    while(!XBeeData.data_received_flag); //wait for Rx to complete
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0) {
             // Reset flag and buffer
    	XBeeData.data_received_flag = 0;
       memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
        	 // Send the ATSL command
       HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
        	 // Receive the response (Serial Number Low)
       HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
       while(!XBeeData.data_received_flag); //wait for Rx to complete
        	 // Check response for ATWR
       if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0) {
        // Handle memory write failure
        printf("Failed to write changes to memory!\n");
        }
        	  }
}

void setDestinationAddress(uint32_t DH, uint32_t DL)
{
    char at_high[20];
    char at_low[20];
    char command_mode[3] = "+++";  //Command to enter AT command mode
    char exit_command[] = "ATCN\r";  // Command to exit AT command mode

    // Format the AT commands
    snprintf(at_high, sizeof(at_high), "ATDH %08X\r", (unsigned int)DH);
    snprintf(at_low, sizeof(at_low), "ATDL %08X\r", (unsigned int)DL);

    // Clear rx_buffer and reset the data_received_flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;

    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    HAL_UART_Receive_IT(&huart1,  &XBeeData.received_byte, 1);

	 while(!XBeeData.data_received_flag); //wait for Rx to complete
    // Transmit ATDH command
    HAL_UART_Transmit(&huart1, (uint8_t *)at_high, strlen(at_high), HAL_MAX_DELAY);

    // Enable reception interrupt
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    // Wait for reception to complete
    while (!XBeeData.data_received_flag);

    // Check response for ATDH
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0) {
        // Reset flag and buffer
    	XBeeData.data_received_flag = 0;
        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);

        // Transmit ATDL command
        HAL_UART_Transmit(&huart1, (uint8_t *)at_low, strlen(at_low), HAL_MAX_DELAY);

        // Enable reception interrupt
        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

        // Wait for reception to complete
        while (!XBeeData.data_received_flag);

        // Check response for ATDL
        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0) {
            // Reset flag and buffer
        	XBeeData.data_received_flag = 0;
            memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);

            // exit command
            HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
            HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

            // Wait for reception to complete
            while (!XBeeData.data_received_flag);

            // Check response for ATCN
            if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0) {
                // Handle memory write failure
                printf("Failed to exit!\n");
            }
        } else {
            // Handle ATDL failure
            printf("Failed to set destination low address!\n");
        }
    } else {
        // Handle ATDH failure
        printf("Failed to set destination high address!\n");
    }
}

/**
 * @brief  Change the XBee Transmit Power Level (ATPL command).
 * @param  Level: Power level (0 to 4, where 4 is the highest).
 */
void TxPowerLevel(uint8_t Level)
{
	char PL[10];
    // Clear rx_buffer and reset the data_received_flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
   // char at_command[] = "ATPL2";  // Command to request Serial Number Low
    // Format the AT commands
    snprintf(PL, sizeof(PL), "ATPL %01X\r", (unsigned int)Level);
    char write[] = "ATWR\r";
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)PL, strlen(PL), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
    {
    	XBeeData.data_received_flag = 0;
        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
        HAL_UART_Transmit(&huart1, (uint8_t*)write, strlen(write), HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
        // Wait for reception to complete
        while (!XBeeData.data_received_flag);
        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0)
        {
        	// Handle memory write failure
             printf("Failed to write changes to memory!\n");
       }
    }
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
}

/*
 *
 */
void ATNI(void)
{
	char at_command[] = "ATNI Switch01\r";
    // Clear rx_buffer and reset the data_received_flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;

    char write[] = "ATWR\r";
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
    {
    	XBeeData.data_received_flag = 0;
        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
        HAL_UART_Transmit(&huart1, (uint8_t*)write, strlen(write), HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
        // Wait for reception to complete
        while (!XBeeData.data_received_flag);
        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0)
        {
        	// Handle memory write failure
             printf("Failed to write changes to memory!\n");
       }
    }
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
}


void FactoryReset(){
	// Clear rx_buffer and reset the data_received_flag
	memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	XBeeData.data_received_flag = 0;
	char at_command[] = "ATRE\r";  // Command for factory reset
	char write[] = "ATWR\r";		// Command to write to NVMe
	//send ATPL command
	HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
	while (!XBeeData.data_received_flag);
	if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
	    {
			XBeeData.data_received_flag = 0;
	        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	        HAL_UART_Transmit(&huart1, (uint8_t*)write, strlen(write), HAL_MAX_DELAY);
	        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
	        // Wait for reception to complete
	        while (!XBeeData.data_received_flag);
	        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0)
	        {
	        	// Handle memory write failure
	             printf("Failed to write changes to memory!\n");
	       }
	    }
	    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	    XBeeData.data_received_flag = 0;
}

/**
 * @brief  Request current power level (ATPL command).
 */
void RQPowerLevel()
{
    // Clear rx_buffer and reset the data_received_flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
    char at_command[] = "ATPL\r";  // Command to request Serial Number Low
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
}

/**
 * Node Discovery is used to discover devices within the XBee network.
 * When issued, the local XBee module scans the network and reports details about each discovered device,
 * including its addresses, Node Identifier, parent address, and other parameters.
 * it also increments ny1 the number of devices found
 */
void XBee_NodeDiscovery()
{
	// Clear buffer and reset flag
	  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	  XBeeData.data_received_flag = 0;

	  HAL_UART_Transmit(&huart1, (uint8_t *)"ATND\r", 5, 1000); // send command for ATND
	  HAL_Delay(200);
	  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
	  for(int i = 0; i < MAX_DEVICES; i++){
		  while (!XBeeData.data_received_flag);
		  if(memcmp(XBeeData.rx_buffer, "\r", 1) == 0) break; //end of discovery
		  memcpy(newNode[i].NetAddress, XBeeData.rx_buffer, sizeof(newNode[i].NetAddress)); // store Network Address
		  //clear buffer and reset flag
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].SerialHigh, XBeeData.rx_buffer, sizeof(newNode[i].SerialHigh)); //store Serial # High
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].SerialLow, XBeeData.rx_buffer, sizeof(newNode[i].SerialLow)); //store serial # Low
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].NodeID, XBeeData.rx_buffer, sizeof(newNode[i].NodeID));		//store Node Identifier
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].pAddress, XBeeData.rx_buffer, sizeof(newNode[i].pAddress));		//store parent Address
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].dType, XBeeData.rx_buffer, sizeof(newNode[i].dType));		//store device type
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].RSSI, XBeeData.rx_buffer, sizeof(newNode[i].RSSI));			//store RSSI
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].pID, XBeeData.rx_buffer, sizeof(newNode[i].pID));		//profile ID
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].manID, XBeeData.rx_buffer, sizeof(newNode[i].manID));	//manufacturer ID
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);			//receive and reset carriage return (signifies end of one device spec)
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

		  deviceCount++; //increment # of devices discovered

		    // Check if the maximum device limit is reached
		    if (deviceCount >= MAX_DEVICES)
		    {
		        break;
		    }
	  }
}

/**
 * @brief  Exit XBee AT Command Mode (ATCN command).
 */
void exitCommandMode(void)
{
    char exit_command[] = "ATCN\r";  // Command to exit AT command mode

    // Send ATCN command to exit command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, (uint8_t*)XBeeData.rx_buffer, 3);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
}
