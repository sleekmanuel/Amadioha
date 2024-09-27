/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *  ********************PROJECT DESCRIPTION********************
  *  Firmware for smart light switch. Smart switch communicates with Hub to connecto to the internet
  *  Sends current value (mA) to hub via zigbee network
  *  Receives action command to turn on or off load from end device(motion sensor)
  *  also receives on and off command from Hub (interface from internet)
  *
  *
  *Receive Data frame via Zigbee Protocol
  *Receive 	RxData[6]
  *Receive 		  [0]: ATSL 1
  *Receive 		  [1]: ATSL 2
  *Receive 		  [2]: ATSL 3
  *Receive 		  [3]: ATSL 4
  *Receive 		  [4]: Control -> Command (C0) or Request (FF)
  *Receive 		  [5]: Command Data -> Turn ON Load (0F) Turn off Load (0A)
  *Receive 		  [5]: Request Data -> Current Data (01)
  *
  *Timers:
  *Timers: TIM1 -> used temporarily to test transmission
  *Timers: TIM2 -> utilized for PWM input from TMCS1123
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_REF 3.3 // ADC reference voltage (Vref) in volts
#define ADC_RESOLUTION 4096.0  // ADC resolution (12-bit gives values from 0 to 4095)
#define SENSITIVITY 25.0  // TMCS1123B2A sensitivity (mV per Ampere, example: 50 mV/A)
#define Data_BUFFER_SIZE 12  // Transmission Buffer size
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
///////// XBee PV ////////////////////
uint8_t txData[Data_BUFFER_SIZE];   // Buffer to store XBee transmission
uint8_t loadActive = 0;				 // Status for active Load
volatile uint8_t data_received_flag = 0;  // Flag to indicate data reception
uint8_t rxData;  /// Received data to process
volatile uint8_t rxIndex = 0;   //iterating variable to store received data in buffer
uint8_t rx_buffer[Data_BUFFER_SIZE];             // Buffer to store received data
uint8_t TxRxData[Data_BUFFER_SIZE];

//Xbee transmission dataframe
uint32_t slAddress;				 // source low address
uint8_t Control;                //used to determine if message is a request or command
uint8_t Data;				   // Transmission data

//ADC PV for Current reading
uint8_t txCurrentValue;		// Current Value to transmit over zigbee protoc
uint32_t CurrentRead;		// Read current ADC value


char serial_number[10] = {0};
char response[10] = {0};

//PWM PV for Sensor Diagnostics
volatile uint32_t lastCapture = 0;
volatile uint32_t pwmPeriod = 0;
volatile uint32_t pwmHighTime = 0;
volatile float dutyCycle = 0;
volatile float frequency = 0;
volatile uint8_t isRisingEdge = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Enable_Load(void);
void Disable_Load(void);
uint32_t Parse_RxSLData(uint8_t[]);
uint32_t Read_ADC(void);
uint8_t Calculate_Current(uint32_t);

void enterCommandMode(void);
void requestSerialNumberLow(void);
void exitCommandMode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, sizeof(rx_buffer));

	  //enterCommandMode();

  // Request and store XBee Serial Number Low
	 // requestSerialNumberLow();

  // Exit command mode
	 // exitCommandMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		if(data_received_flag)
		{

			// called parse received data
			slAddress = Parse_RxSLData((char*)TxRxData);
			Control = rx_buffer[4];   // extract command information
			Data = rx_buffer[5];
			if(Control == 0xC0){
				if(Data == 0x0F){
					  if(loadActive){
						  ;
					  }else{
						  Enable_Load();
					  }
				}else if(Data == 0x0F){
					  if(loadActive){
						  Disable_Load();
					  }else{
						  ;
					  }
				}
			}
			data_received_flag = 0;  // resets received status to expect new data
		}

		CurrentRead = Read_ADC();
		txCurrentValue = 0xF0;
				//Calculate_Current(CurrentRead);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Function to read ADC value
uint32_t Read_ADC(void)
{
    HAL_ADC_Start(&hadc1);  // Start ADC conversion
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  // Wait for conversion to finish
    uint32_t adcValue = HAL_ADC_GetValue(&hadc1);  // Get the ADC value
    HAL_ADC_Stop(&hadc1);  // Stop the ADC
    return adcValue;
}

// Function to calculate current based on ADC value
uint8_t Calculate_Current(uint32_t adcValue)
{
    // Convert ADC value to voltage
    float voltage = (adcValue * V_REF) / ADC_RESOLUTION;
    // Sensor outputs 0.5 * Vcc at zero current
    float zeroCurrentVoltage = V_REF / 2.0;
    // Calculate current using sensor sensitivity (mV/A)
    float current = (voltage - zeroCurrentVoltage) * 1000 / SENSITIVITY;  // in Amps
    uint8_t mAmps = current * 1000; //current in mA

    return mAmps;
}
/*
 * Enable Load when Load is disabled
 * Turn on Onboard LED
 */
void Enable_Load(void)
{
	loadActive = 1;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET); //engage relay
	HAL_GPIO_WritePin(GPIOA, VBase_Pin, SET); // turn on LEDs
}
/*
 * Disable Load when Load is active
 * Turn off Onboard LED
 */
void Disable_Load(void)
{
	loadActive = 0;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET); // disengage relay
	HAL_GPIO_WritePin(GPIOA, VBase_Pin, RESET); // turn off LEDs
}
/*
 * Parse Received Data for ATSL
 *
 */
uint32_t Parse_RxSLData(uint8_t data[])
{
	uint32_t address = 0;
    // Shift and merge the 4 bytes into a 32-bit integer
    address |= ((uint32_t)data[0] << 24);  // Most significant byte
    address |= ((uint32_t)data[1] << 16);
    address |= ((uint32_t)data[2] << 8);
    address |= ((uint32_t)data[3] << 0);   // Least significant byte

    return address;
}

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Check if the power on button is pushed
   * if pushed when the load is active, it turns off the load and vice-versa
   *  */
  if((GPIO_Pin = Switch_Pin)){
	  if(loadActive){
		  Disable_Load();
	  }else{
		  Enable_Load();
	  }

  }


}

 void enterCommandMode(void)
 {
     char command_mode[4] = "+++";


     // Send "+++" to enter AT command mode
     HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
     HAL_Delay(500);  // Small delay for XBee to respond

     // Receive the "OK" response from XBee
     HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, Data_BUFFER_SIZE);

     if(data_received_flag){

     }

     // Check if response is "OK"
     if (strstr(response, "OK") != NULL)
     {
         // Successfully entered command mode
     }
     else
     {
         // Failed to enter command mode
     }
 }

 // Function to request XBee Serial Number Low (ATSL)
 void requestSerialNumberLow(void)
 {
     char at_command[] = "ATSL\r";  // Command to request Serial Number Low
     //char serial_number[20] = {0};  // Buffer to store the Serial Number Low

     // Send the ATSL command
     HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);

     // Receive the response (Serial Number Low)
     HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, Data_BUFFER_SIZE);

     // Store or process the received Serial Number Low
     // Example: Print it via UART or store it in memory
     printf("Serial Number Low: %s\r\n", serial_number);
 }

 // Function to exit XBee AT Command Mode
 void exitCommandMode(void)
 {
     char exit_command[] = "ATCN\r";  // Command to exit AT command mode

     // Send ATCN command to exit command mode
     HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
     HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, Data_BUFFER_SIZE);
     rxIndex = 0;
 }
 /*
  * Receive interrupt callback function
  */

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
     if (huart->Instance == USART1)  // Ensure it's USART1
     {
    	 data_received_flag = 1;    // Indicate data has been received

    	 memcpy(TxRxData, rx_buffer, Data_BUFFER_SIZE);  // Move the received data to the transmission buffer
    	 memset(rx_buffer, 0, Data_BUFFER_SIZE); // Optionally clear the rx_buffer

         HAL_UART_Receive_IT(&huart1, rx_buffer, Data_BUFFER_SIZE);   // Re-enable receiving more data
     }

     // Handle Overrun Error
     if (USART1->ISR & USART_ISR_ORE)
     {
         // Read status register to clear ORE flag
         uint32_t temp = USART1->ISR;
         // Read data register to clear the ORE flag
         (void)USART1->RDR;
         // Re-enable UART receive interrupt
         HAL_UART_Receive_IT(&huart1, rx_buffer, Data_BUFFER_SIZE);
     }
 }

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
   if (htim->Instance == TIM2)  // Check if the interrupt is from TIM1
   {
     /* Transmit current value after .1 sec */
	 //HAL_UART_Transmit(&huart1, &txCurrentValue, sizeof(txCurrentValue), HAL_MAX_DELAY);
	   //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
   }
 }




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
