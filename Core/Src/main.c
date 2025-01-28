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
 **********************PROJECT DESCRIPTION***********************************
  *  Firmware for smart light switch. Smart switch communicates with Hub to connecto to the internet
  *  Sends current value (A) to hub via zigbee network
  *  Receives action command to turn on or off load from end device(motion sensor)
  *  also receives on and off command from Hub (interface from internet)
  ******************************************************************************
  *
  * Data frame via Zigbee Protocol
  *Receive  	   [11]
  *Receive 		   [0]: ATSL 1
  *Receive 		   [1]: ATSL 2
  *Receive 		   [2]: ATSL 3
  *Receive 		   [3]: ATSL 4
  *Receive 		   [4]: ATSL 5
  *Receive 		   [5]: ATSL 6
  *Receive 		   [6]: ATSL 7
  *Receive 		   [7]: ATSL 8
  *Receive 		   [8]: Control -> Command (C0) or Request (FF)
  *Receive 		   [9]: Command Data -> Turn ON Load (0F) Turn off Load (0A)
  *Receive 		   [10]: Request Data -> Current Data (01)
  *Receive 		   [11]: '\r' End of Message
  *
  *Timers:
  *Timers:
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
#include "math.h"
#include <zigbee.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
///////// XBee PV ////////////////////
uint8_t loadActive = 0;				 // Status for active Load
uint8_t previousLoadActive = 0;		// Store previous load state
uint8_t Load_Active[11] = {52, 50, 55, 48, 55, 53, 55, 65, 0xB3, 0x11, 0x0D};	// load active feedback
uint8_t Load_Inactive[11] = {52, 50, 55, 48, 55, 53, 55, 65, 0xB3, 0xAA, 0x0D}; // load inactive feedback

//Xbee transmission dataframe

 XBeeModule XBeeData =
 {		//initialization
		.rx_buffer = {0},
		.received_byte = 0,
		.data_received_flag = 0,
		.overflow_flag = 0,
		.myAddress = {0}
};
 NodeDiscovery newNode[MAX_DEVICES];

uint8_t myDestLow[8];			// store destination address low
uint8_t Control;                //used to determine if message is a request or command
uint8_t Data;				   // Transmission data
uint8_t deviceCount = 1;		// # of devices in network. initialized with 1 to count current device

//ADC PV for Current reading
float currentRMS = 0;  // Read current RMS value
volatile uint16_t adcValue;
volatile uint32_t lastDebounceTime = 0;
float samples[SAMPLE_COUNT];
char buffer[32];
char currentResponse[CURRENT_RESPONSE_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Enable_Load(void);
void Disable_Load(void);
void CheckAndTransmitLoadChange(void);
uint32_t Parse_RxSLData(uint8_t[]);
float Read_ADC(void);
float Calculate_RMS(float samples[], int sampleCount);
void handleSwitchControl(uint8_t Data);
void handleCurrentControl(void);
void RQPowerLevel();
void TxPowerLevel(uint8_t Level);
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
  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);  // start IT receiving


/* XBEE Configuration--------------------------------------------------------*/
//
  //
  //enterCommandMode();
 // factoryReset();
  //XBee_NodeDiscovery();  // discover all devices in the same network. store in newNode
  //exitCommandMode();
 // HAL_Delay(2000);
  //requestParameter("ATdb\r", myDestLow, sizeof(myDestLow));
  //setDestinationAddress(0x00, 0xFFFF);
  //setParameter("ATSC0010\r");
 // Request and store XBee Serial Number Low
  enterCommandMode();
   if (requestParameter("ATSL\r", XBeeData.myAddress, sizeof(XBeeData.myAddress)) == XBEE_SUCCESS) {
  } else {
	  //Blink LED in a loop to indicate issue
      while(1){
    	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // turn on LEDs
    	  HAL_Delay(100);
      }
  }

/* XBEE Configuration Ends--------------------------------------------------*/
  /* USER CODE END 2 */
  memcpy(currentResponse, XBeeData.myAddress, 8);  //append my serial # low to current value response variable
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*        BLICK TWICE FOR SUCCESSFUL INITIALIZATION       */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  for(int j = 0; j < 2; j++)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(500);
  }


  while (1)
  {

	  if (XBeeData.data_received_flag) {
	           // Check if the message is meant for me
	           if (memcmp(XBeeData.myAddress, XBeeData.rx_buffer, 8) == 0) {
	               // Extract and handle command and data
	               Control = XBeeData.rx_buffer[8];
	               Data = XBeeData.rx_buffer[9];

	               if (Control == 0xC0) {
	                   handleSwitchControl(Data);
	               } else if (Control == 0xFF && Data == 0x01) {
	                   handleCurrentControl();
	               }
	           }

	           // Reset flags and buffers after processing
	           XBeeData.data_received_flag = 0;
	           memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	           HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
	       }

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
float Read_ADC(void)
{
    HAL_ADC_Start(&hadc1);  // Start ADC conversion
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  // Wait for conversion to finish
     adcValue = HAL_ADC_GetValue(&hadc1);  // Get the ADC value

    volatile float voltage = (adcValue / ADC_RESOLUTION) * V_REF;  // Convert ADC value to voltage
    float zeroCurrentVoltage = 2.45;      // Measured sensor output voltage at zero current
    // Calculate current using sensor sensitivity (mV/A)
    float current = (voltage - zeroCurrentVoltage) / SENSITIVITY;  // in Amps
    return current;
}
// Function to calculate the RMS value of the sampled current
float Calculate_RMS(float samples[], int sampleCount) {
    float sumSquares = 0;

    for (int i = 0; i < sampleCount; i++) {
        sumSquares += samples[i] * samples[i];
    }

    // Return the square root of the mean of the squares (RMS value)
    return sqrt(sumSquares / sampleCount);
}

/*
 * checks if load changed state and transmits new state to motion sensor
 */
void CheckAndTransmitLoadChange(void)
{
    if (loadActive != previousLoadActive) {

        if (loadActive) {
            HAL_UART_Transmit(&huart1, Load_Active, sizeof(Load_Active), HAL_MAX_DELAY);  // Send "load active" message
        } else {
            HAL_UART_Transmit(&huart1, Load_Inactive, sizeof(Load_Inactive), HAL_MAX_DELAY);  // Send "load inactive" message
        }

        previousLoadActive = loadActive;
    }
}
/*
 * @func processes information for load control
 * @param Data section of Rx transmission
 */
void handleSwitchControl(uint8_t Data)
{
    if (Data == 0x0F) {
        if (!loadActive) {
            Enable_Load();
        }
    } else if (Data == 0x0A) {
        if (loadActive) {
            Disable_Load();
        }
    }
}

// Function to handle Control == 0xFF and Data == 0x01
void handleCurrentControl()
{
    // Take sample of SAMPLE_COUNT of current readings
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = Read_ADC();
        HAL_Delay(1);
    }
    currentRMS = Calculate_RMS(samples, SAMPLE_COUNT); // Calculate the RMS value
    sprintf(buffer, "%.2f\r", currentRMS); // Format float to a string

    int msgLength = strlen((char*)currentResponse);
    int bufferLength = strlen(buffer);
    if (msgLength + bufferLength  < CURRENT_RESPONSE_SIZE){
        strcat((char *)currentResponse, buffer); // Append currentRMS to xbeeMessage
    }
    HAL_UART_Transmit(&huart1,  (uint8_t *)currentResponse, strlen((char*)currentResponse), HAL_MAX_DELAY);

    // Reset the currentResponse to its initial state
    memset(currentResponse + 8, 0, CURRENT_RESPONSE_SIZE - 8); // Clear all content after the 8-byte address
    currentResponse[8] = '\0'; // Ensure null termination
}

/*
 * Enable Load when Load is disabled
 * Turn on Onboard LED
 */
void Enable_Load(void)
{
	loadActive = 1;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET); // turn on LEDs
	HAL_GPIO_WritePin(GPIOA, VBase_Pin, SET); //engage relay
	CheckAndTransmitLoadChange();
}
/*
 * Disable Load when Load is active
 * Turn off Onboard LED
 */
void Disable_Load(void)
{
	loadActive = 0;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET); // turn off LEDs
	HAL_GPIO_WritePin(GPIOA, VBase_Pin, RESET); // disengage relay
	CheckAndTransmitLoadChange();
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

/* Check if the power on button is pushed
 * if pushed when the load is active, it turns off the load and vice-versa
*/
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	 if(GPIO_Pin == Switch_Pin)
	     {
	         /* Get the current time (in milliseconds) */
	         uint32_t currentTime = HAL_GetTick(); // HAL_GetTick() returns the system time in ms
	         /* Check if enough time has passed since the last press to consider this a valid press */
	         if((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY_MS)
	         {
	             /* Toggle the load state */
	             (loadActive) ? Disable_Load(): Enable_Load();
	             /* Update the last debounce time */
	             lastDebounceTime = currentTime;
	         }
	     }
}


 /*
  * Receive interrupt callback function
  */

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
     static uint8_t index = 0;

     if (huart->Instance == USART1) {
         if (index < DATA_BUFFER_SIZE - 1) {
        	 XBeeData.rx_buffer[index++] = XBeeData.received_byte;

             if (XBeeData.received_byte == '\r') {  // End of response
            	 XBeeData.data_received_flag = 1;
                 XBeeData.rx_buffer[index] = '\0';  // Null-terminate
                 index = 0;  // Reset for next reception
             }
         } else {
        	 XBeeData.overflow_flag = 1;  // Signal buffer overflow
             index = 0;  // Optionally reset the buffer
         }

         HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);  // Continue receiving
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
