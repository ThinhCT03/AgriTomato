/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lora.h"
#include <stdio.h>
#include "config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
LoRa myLoRa;
uint16_t LoRa_Status;

typedef struct {
    uint8_t Check_ID;
    uint16_t Humidity;
    uint16_t Temperature;
    uint16_t LUX;
    uint16_t Data_MQ135;
    uint16_t soil_Temp;
    uint16_t soil_Moisture;
} SensorData_t;
SensorData_t sensorData;

uint8_t Received_Data[16]; //Buffer to store received data
volatile uint8_t relay_state = 0; // Variable to hold the relay state extracted from received data
volatile uint8_t current_state_flag = 0; //flag to send relays state to Gateway
uint8_t Alarm_set = 0x03;
uint8_t Transmit_Status_Data[2];
volatile uint32_t relayFanEndTime;
volatile uint32_t relayPumpEndTime;

typedef enum {
    MODE_MANUAL,
    MODE_AUTOMATE
} ControlMode_t;
ControlMode_t control_mode = MODE_MANUAL;

#define CMD_MANUAL   0xAA
#define CMD_AUTOMATE 0xBB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

static void Relay_Change_State(void);
static void Fuzzilization_Data(void);
void Fuzzy_Pump_Control(void);
void Fuzzy_Fan_Control(void);
void Fuzzy_Check_Relay_Status(void);

float Critical_High_Temp(uint16_t Temp);
float Critical_Low_Temp(uint16_t Temp);
float Critical_High_Hum(uint16_t Hum);
float Critical_Low_Hum(uint16_t Hum);
float Critical_High_MQ135(uint16_t MQ135);
float Critical_High_Moisture(uint16_t Moisture);
float Critical_Low_Moisture(uint16_t Moisture);
float Critical_High_TempSoil(uint16_t TempSoil);
float Critical_Low_TempSoil(uint16_t TempSoil);
void Light_Control(uint16_t LUX);

void Set_Control_Mode_From_Command(uint8_t cmd);
void Send_Relay_Mode_Status(void);
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //LORA STATUS CONFIG
  myLoRa = newLoRa();

  myLoRa.CS_port         = NSS_GPIO_Port;
  myLoRa.CS_pin          = NSS_Pin;
  myLoRa.reset_port      = RST_GPIO_Port;
  myLoRa.reset_pin       = RST_Pin;
  myLoRa.DIO0_port       = DIO0_GPIO_Port;
  myLoRa.DIO0_pin        = DIO0_Pin;
  myLoRa.hSPIx           = &hspi1;

  myLoRa.frequency             = 440;             // default = 433 MHz
  myLoRa.spredingFactor        = SF_7;            // default = SF_7
  myLoRa.bandWidth             = BW_31_25KHz;       // default = BW_125KHz
  myLoRa.crcRate               = CR_4_5;          // default = CR_4_5
  myLoRa.power                 = POWER_20db;      // default = 20db
  myLoRa.overCurrentProtection = 130;             // default = 100 mA
  myLoRa.preamble              = 9;              // default = 8;

  if (LoRa_init(&myLoRa) == LORA_OK)
  {
  	LoRa_Status = 1;
  }

  LoRa_startReceiving(&myLoRa);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if new_function == 1
	  Send_Relay_Mode_Status();// Hàm này chỉ được gọi một lần khi RTC wakeup mỗi 12s

	  if (Received_Data[0] == 0xFF)
	  {
		  //TURNOFF ALL THE RELAYS WHEN GATEWAY OFFER RESTART THE NETWORK
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);
		  while(Received_Data[0] != 0xC0) //0XEE
		  {

		  }
		  HAL_Delay(750); //Wait for the other 2 nodes complete transmit
		  LoRa_transmit(&myLoRa, &Alarm_set, sizeof(Alarm_set), 500);
		  LoRa_startReceiving(&myLoRa);
		  while(Received_Data[0] != 0xB0) //0xDD
		  {

		  }
		  MX_RTC_Init();
	  }
	  else
	  {
		  switch (control_mode)
		  {
		      case MODE_MANUAL:
		          Relay_Change_State();
		          break;

		      case MODE_AUTOMATE:
				  Fuzzilization_Data();
				  Fuzzy_Check_Relay_Status();
				  break;

		      default:
		    	  break;
		  }
	  }
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x12;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, light_relay_Pin|water_relay_Pin|fan_relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NSS_Pin RST_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : light_relay_Pin water_relay_Pin fan_relay_Pin */
  GPIO_InitStruct.Pin = light_relay_Pin|water_relay_Pin|fan_relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : light_sw_Pin water_sw_Pin fan_sw_Pin */
  GPIO_InitStruct.Pin = light_sw_Pin|water_sw_Pin|fan_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//THIS WAKE UP TRIGGERED BY GPIO DIO0, HAPPENS WHEN A DATA RECEIVED BY SX1278 MODULE
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO0_Pin)
    {
    	LoRa_receive(&myLoRa, Received_Data, sizeof(Received_Data));
    	Set_Control_Mode_From_Command(Received_Data[1]);
    }
}


#if new_function == 1
static void Relay_Change_State(void)
{
    if (Received_Data[0] == 0xA3 && Received_Data[1] == 0xAA)
    {
        // Extract relay state from the second byte
        relay_state = Received_Data[2];

        // Control the relays based on the relay state
        if (relay_state & 0x01)  // Check if bit 0 (water pump) is set
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // Turn on water pump
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // Turn off water pump
        }

        if (relay_state & 0x02)  // Check if bit 1 (light) is set
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // Turn on light
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Turn off light
        }

        if (relay_state & 0x04)  // Check if bit 2 (fan) is set
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Turn on fan
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Turn off fan
        }
    }
}

void Set_Control_Mode_From_Command(uint8_t cmd)
{
    switch(cmd)
    {
        case CMD_MANUAL:
            control_mode = MODE_MANUAL;
            break;
        case CMD_AUTOMATE:
            control_mode = MODE_AUTOMATE;
            break;
        default:
            // Xử lý nếu cần: giữ nguyên mode cũ hoặc báo lỗi
            break;
    }
}


void Send_Relay_Mode_Status(void)
{
	  if(current_state_flag == 1)
	  {
		  //transmit back to gateway
		  Transmit_Status_Data[0] = 0xA3; // ID byte
		  uint8_t pa9 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		  uint8_t pa10 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
		  uint8_t pa11 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);

		  // Gộp trạng thái lại thành 1 byte:
		  // Ví dụ: PA11 -> bit 2, PA10 -> bit 1, PA9 -> bit 0
		  Transmit_Status_Data[1] = (pa11 << 2) | (pa10 << 1) | pa9;
		  LoRa_transmit(&myLoRa, Transmit_Status_Data, sizeof(Transmit_Status_Data), 500);
		  LoRa_startReceiving(&myLoRa);
		  current_state_flag = 0;
		  //return to receive state
	  }
}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_AlarmTypeDef sAlarm = {0};

    // Going to send back current state
    current_state_flag = 1;

    // Get current time
    HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);  // Chuyển sang RTC_FORMAT_BIN để dễ tính toán

    // Adding 10s to wake up sequence, because first setting of the alarm is 10s so this node will wake up at 10s 20s 30s...
    uint32_t total_seconds = sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds + 10;
    sAlarm.AlarmTime.Hours   = (total_seconds / 3600) % 24;
    sAlarm.AlarmTime.Minutes = (total_seconds / 60) % 60;
    sAlarm.AlarmTime.Seconds = total_seconds % 60;
    sAlarm.Alarm = RTC_ALARM_A;
    // Re-set alarm with updated wakeup time
    HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, RTC_FORMAT_BIN);  // Sử dụng RTC_FORMAT_BIN
}





//----------------------------------------------------------------------------FUZZY CONTROL HANDLE FUNCTIONS-----------------------------------------------------------------------------------------------------------//
static void Fuzzilization_Data(void)
{
	if((Received_Data[2] == 0x31) && (Received_Data[3] == 0x32))
	{
		sensorData.Humidity    = Received_Data[5] | (Received_Data[4] << 8);
		sensorData.Temperature = Received_Data[7] | (Received_Data[6] << 8);
		sensorData.LUX         = Received_Data[9] | (Received_Data[8] << 8);
		sensorData.Data_MQ135  = Received_Data[11] | (Received_Data[10] << 8);
	    sensorData.soil_Temp = Received_Data[13] | (Received_Data[12] << 8);
	    sensorData.soil_Moisture = Received_Data[15] | (Received_Data[14] << 8);

	    Fuzzy_Fan_Control();
	    Fuzzy_Pump_Control();
	    Light_Control(sensorData.LUX);
	}
}

void Fuzzy_Check_Relay_Status(void)
{
    if (HAL_GetTick() >= relayPumpEndTime)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);  // Relay OFF
    }

    else if (HAL_GetTick() >= relayFanEndTime)
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // Relay OFF
    }
}

void Light_Control(uint16_t LUX)
{
    if (LUX <= 2000)
    {
        // TurnOff_Light();
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // Turn on light
    }
    else
    {
        // TurnOn_Light();
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Turn off light
    }
}

void Fuzzy_Fan_Control(void)
{
	uint8_t seconds;
	float HUM, MQ135, TEMP;
	HUM = Critical_High_Hum(sensorData.Humidity) * 10;
	MQ135 = Critical_High_MQ135(sensorData.Data_MQ135) * 10;
	TEMP = Critical_High_Temp(sensorData.Temperature) * 10;
	seconds = (int)(0.2*HUM + 0.3*MQ135 + 0.5*TEMP);
    if (seconds > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // Relay ON
        relayFanEndTime = HAL_GetTick() + (seconds * 1000); // lưu thời gian tắt (ms)
    }
}

void Fuzzy_Pump_Control(void)
{
	uint8_t seconds;
	float MOIS, TEMPSOIL, HUM;
	MOIS = Critical_Low_Moisture(sensorData.soil_Moisture) * 10;
	TEMPSOIL = Critical_High_TempSoil(sensorData.soil_Temp) * 10;
	HUM = Critical_Low_Hum(sensorData.Humidity) * 10;
	seconds = (int)(0.2*HUM + 0.2*TEMPSOIL + 0.6*MOIS);
    if (seconds > 0)
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);  // Relay ON
        relayPumpEndTime = HAL_GetTick() + (seconds * 1000); // lưu thời gian tắt (ms)
    }
}



float Critical_High_Hum(uint16_t Hum)
{
	Hum = Hum / 10.0;
    if (Hum < 85) return 0;
    else if (Hum >= 95) return 1;
    else return (Hum - 85) / 10.0;
}
float Critical_Low_Hum(uint16_t Hum)
{
	Hum = Hum / 10.0;
    if (Hum >= 75) return 0;
    else if (Hum <= 60) return 1;
    else return (75 - Hum) / 15.0;
}


float Critical_High_Temp(uint16_t Temp)
{
	Temp = Temp / 10.0;
    if (Temp <= 30) return 0;
    else if (Temp >= 40) return 1;
    else return (Temp - 30) / 10.0;
}
float Critical_Low_Temp(uint16_t Temp)
{
	Temp = Temp / 10.0;
    if (Temp >= 15) return 0;
    else if (Temp <= 8) return 1;
    else return (15 - Temp) / 7.0;
}


float Critical_High_Moisture(uint16_t Moisture)
{
    if (Moisture <= 85) return 0;
    else if (Moisture >= 95) return 1;
    else return (Moisture - 85) / 10;
}
float Critical_Low_Moisture(uint16_t Moisture)
{
    if (Moisture <= 50) return 1;
    else if (Moisture >= 65) return 0;
    else return (65 - Moisture) / 15;
}


float Critical_High_TempSoil(uint16_t TempSoil)
{
    if (TempSoil <= 28) return 0;
    else if (TempSoil >= 38) return 1;
    else return (TempSoil - 28) / 10;
}
float Critical_Low_TempSoil(uint16_t TempSoil)
{
    if (TempSoil <= 10) return 1;
    else if (TempSoil >= 20) return 0;
    else return (20 - TempSoil) / 10;
}

float Critical_High_MQ135(uint16_t MQ135)
{
    if (MQ135 <= 700) return 0;
    else if (MQ135 >= 1200) return 1;
    else return (MQ135 - 700) / 500;
}
#endif
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
