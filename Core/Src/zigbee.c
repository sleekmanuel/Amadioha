/*
 * zigbee.c
 *
 *  Created on: Dec 2, 2024
 *      Author: root
 */



#include <zigbee.h>

// External declarations for UART handle and buffers
extern UART_HandleTypeDef huart1;          // UART handle
extern uint8_t rx_buffer[Data_BUFFER_SIZE]; // Buffer to store received data
extern uint8_t received_byte;              // Variable to store single received byte
extern uint8_t mySerialLow[8];             // Array to store Serial Number Low
extern uint8_t myDestLow[8];               // Array to store Destination Number Low
extern volatile uint8_t data_received_flag; // Flag to indicate data reception completion

/*
int XBEE_SUCCESS;
int XBEE_ERROR_RESPONSE;
*/

/*
 * Enter AT command mode
 * Request XBee Serial Number Low (ATSL)
 * Exit AT command mode
 */


<<<<<<< HEAD
void requestSerialNumberLow(void)
{
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    char command_mode[3] = "+++";  //Command to enter AT command mode
    char at_command[] = "ATSL\r";  // Command to request Serial Number Low
    char exit_command[] = "ATCN\r";  // Command to exit AT command mode

    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    HAL_UART_Receive_IT(&huart1,  &received_byte, 1);

    while(!data_received_flag); //wait for Rx to complete
    if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
        // Reset flag and buffer
        data_received_flag = 0;
        memset(rx_buffer, 0, Data_BUFFER_SIZE);
   	 // Send the ATSL command
   	 HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
   	 // Receive the response (Serial Number Low)
   	 HAL_UART_Receive_IT(&huart1, &received_byte, 1);
   	 while(!data_received_flag); //wait for Rx to complete
   	 memcpy(mySerialLow, rx_buffer, 8);  // Move the received data to the transmission buffer
   	 data_received_flag = 0; //reset receive flag
    }
    // Send ATCN command to exit command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);
    while(!data_received_flag); //wait for Rx to complete
    data_received_flag = 0;
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
}
// Function to request XBee Serial Number Low (ATSL)
void requestDestNumberLow(void)
{
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    char command_mode[3] = "+++";  //Command to enter AT command mode
    char at_command[] = "ATDL\r";  // Command to request Destination Number Low
    char exit_command[] = "ATCN\r";  // Command to exit AT command mode

    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    HAL_UART_Receive_IT(&huart1,  &received_byte, 1);

    while(!data_received_flag); //wait for Rx to complete
    if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
        // Reset flag and buffer
        data_received_flag = 0;
        memset(rx_buffer, 0, Data_BUFFER_SIZE);
   	 // Send the ATSL command
   	 HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
   	 // Receive the response (Serial Number Low)
   	 HAL_UART_Receive_IT(&huart1, &received_byte, 1);
   	 while(!data_received_flag); //wait for Rx to complete
   	 memcpy(myDestLow, rx_buffer, 8);  // Move the received data to the transmission buffer
   	 data_received_flag = 0; //reset receive flag
    }
    // Send ATCN command to exit command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);
}

=======
int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length) {
    // Clear buffer and reset flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
>>>>>>> refs/heads/Dec_13

<<<<<<< HEAD
=======
    char command_mode[] = "+++";
    char exit_command[] = "ATCN\r";

    // Enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t *)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);

    while (!data_received_flag);
    if (strncmp((char *)rx_buffer, "OK", 2) != 0) return XBEE_ERROR_RESPONSE;

    // Send the parameter request command
    data_received_flag = 0;
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);

    while (!data_received_flag);
    if (strlen((char *)rx_buffer) < length) return XBEE_ERROR_RESPONSE;
    memcpy(output_buffer, rx_buffer, length);

    // Exit AT command mode
    data_received_flag = 0;
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);

    while (!data_received_flag);
    return strncmp((char *)rx_buffer, "OK", 2) == 0 ? XBEE_SUCCESS : XBEE_ERROR_RESPONSE;
}

>>>>>>> refs/heads/Dec_13

void writeCommand(){
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    char command_mode[3] = "+++";  //Command to enter AT command mode
    char at_command[] = "ATWR\r";  // Command to write to XBEE EEPROM

    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    HAL_UART_Receive_IT(&huart1,  &received_byte, 1);

    while(!data_received_flag); //wait for Rx to complete
    if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
             // Reset flag and buffer
       data_received_flag = 0;
       memset(rx_buffer, 0, Data_BUFFER_SIZE);
        	 // Send the ATSL command
       HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
        	 // Receive the response (Serial Number Low)
       HAL_UART_Receive_IT(&huart1, &received_byte, 1);
       while(!data_received_flag); //wait for Rx to complete
        	 // Check response for ATWR
       if (strncmp((char *)rx_buffer, "OK", 2) != 0) {
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
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    HAL_UART_Receive_IT(&huart1,  &received_byte, 1);

	 while(!data_received_flag); //wait for Rx to complete
    // Transmit ATDH command
    HAL_UART_Transmit(&huart1, (uint8_t *)at_high, strlen(at_high), HAL_MAX_DELAY);

    // Enable reception interrupt
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);

    // Wait for reception to complete
    while (!data_received_flag);

    // Check response for ATDH
    if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
        // Reset flag and buffer
        data_received_flag = 0;
        memset(rx_buffer, 0, Data_BUFFER_SIZE);

        // Transmit ATDL command
        HAL_UART_Transmit(&huart1, (uint8_t *)at_low, strlen(at_low), HAL_MAX_DELAY);

        // Enable reception interrupt
        HAL_UART_Receive_IT(&huart1, &received_byte, 1);

        // Wait for reception to complete
        while (!data_received_flag);

        // Check response for ATDL
        if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
            // Reset flag and buffer
            data_received_flag = 0;
            memset(rx_buffer, 0, Data_BUFFER_SIZE);

            // exit command
            HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
            HAL_UART_Receive_IT(&huart1, &received_byte, 1);

            // Wait for reception to complete
            while (!data_received_flag);

            // Check response for ATCN
            if (strncmp((char *)rx_buffer, "OK", 2) != 0) {
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

