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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_REF 3.3 // ADC reference voltage (Vref) in volts
#define ADC_RESOLUTION 4096.0  // ADC resolution (12-bit gives values from 0 to 4095)
#define SENSITIVITY 50.0  // TMCS1123B2A sensitivity (mV per Ampere, example: 50 mV/A)
#define TxData_BUFFER_SIZE 6  // Transmission Buffer size
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
///////// XBee PV ////////////////////
uint8_t txData[TxData_BUFFER_SIZE];   // Buffer to store XBee transmission
uint8_t loadActive = 0;				 // Status for active Load
volatile uint8_t data_received_flag = 0;  // Flag to indicate data reception
uint8_t rxData;  /// Received data to process
volatile uint8_t rxIndex = 0;   //iterating variable to store received data in buffer
char rx_buffer[6];             // Buffer to store received data

//Xbee transmission dataframe
uint32_t slAddress;				 // source low address
uint8_t Control;                //used to determine if message is a request or command
uint8_t Data;				   // Transmission data

//ADC PV for Current reading
uint8_t txCurrentValue;		// Current Value to transmit over zigbee protoc
uint32_t CurrentRead;		// Read current ADC value


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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		if(data_received_flag)
		{
			// called parse received data
			slAddress = Parse_RxSLData((uint8_t*)rx_buffer);
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
		txCurrentValue = Calculate_Current(CurrentRead);
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
 /*
  * Receive interrupt callback function
  */

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {

 	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))  // Check if data is received
 	    {
 	        char received_char = (uint8_t)(huart1.Instance->RDR & 0xFF);  // Read the received character
 	        if (rxIndex < sizeof(rx_buffer) - 1) {
 	            rx_buffer[rxIndex++] = received_char;
 	        }

 	        // Check for newline or carriage return as end of response
 	        if ((rxIndex == sizeof(rx_buffer)) || received_char == '\r') {
 	            rx_buffer[rxIndex] = '\0';  // Null terminate the string
 	            rxIndex = 0;
 	            data_received_flag = 1;  // Set flag to indicate that data is fully received
 	        }

 	    }
 	 HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 6);
 }

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
   if (htim->Instance == TIM1)  // Check if the interrupt is from TIM1
   {
     /* Transmit current value after 1 min */
	  HAL_UART_Transmit(&huart1, &txCurrentValue, sizeof(txCurrentValue), HAL_MAX_DELAY);
   }
 }


 /*
  * Timer to measure duty cycle PMW input for Sensor Diagnostics from TMCS1123
  * 100% -> No Fault
  * 20% -> Thermal & Sensor Fault
  * 50% -> Sensor Fault
  * 80% -> Thermal fault
  */
 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 {
     if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
     {
         uint32_t captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

         if (isRisingEdge) // Rising edge detected
         {
             pwmPeriod = captureValue - lastCapture;  // Calculate period
             lastCapture = captureValue;
             isRisingEdge = 0;  // Switch to falling edge
             __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
         }
         else  // Falling edge detected
         {
             pwmHighTime = captureValue - lastCapture;  // High time (on time)
             lastCapture = captureValue;
             isRisingEdge = 1;  // Switch to rising edge
             __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

             // Calculate Duty Cycle and Frequency
             dutyCycle = ((float)pwmHighTime / (float)pwmPeriod) * 100.0;
             frequency = HAL_RCC_GetPCLK1Freq() / (htim2.Init.Prescaler + 1) / pwmPeriod;
         }
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
