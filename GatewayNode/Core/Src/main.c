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
#include "lcd-i2c.h"
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
uint8_t TxBuffer[4];
uint8_t Received_Data[9];
uint8_t RX_Data_Control[16];
uint8_t receivedACK[3];
volatile uint8_t Start = 1;
volatile uint8_t Data_Sensor_Interrupt = 0;
uint8_t ESP_rx_data;
volatile uint8_t manual_interrupt_flag = 0;
volatile uint8_t automate_interrupt_flag = 0;

//RELAYS STATE VARIABLES
volatile uint8_t water_relay = 0; // 0: OFF, 1: ON
volatile uint8_t light_relay = 0; // 0: OFF, 1: ON
volatile uint8_t fan_relay = 0; // 0: OFF, 1: ON
volatile uint8_t relay_state = 0;

volatile uint8_t Next_State_Flag = 0;
volatile uint8_t Prev_State_Flag = 0;

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

typedef enum {
    MODE_MANUAL,
    MODE_AUTOMATE
} ControlMode_t;
ControlMode_t control_mode = MODE_MANUAL;

typedef enum {
    SCREEN_SENSOR = 0,
    SCREEN_RELAY,
    SCREEN_MODE,
    SCREEN_TOTAL
} DisplayScreen_t;
DisplayScreen_t current_display_screen = SCREEN_SENSOR;

uint8_t buffer[14];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void LCD_Display(SensorData_t *sensorData);
void Received_Data_Handler(void);
void Sendto_ESP_UART(SensorData_t *data);
void change_relay_state(void);
void change_relay_state_update_manual(void);

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  water_relay = 0; // 0: OFF, 1: ON
  light_relay = 0; // 0: OFF, 1: ON
  fan_relay = 0; // 0: OFF, 1: ON
  relay_state = 0;

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

  if (LoRa_init(&myLoRa) == LORA_OK){
	  LoRa_Status = 1;
  }

  lcd_put_cur(1, 0);
  lcd_send_string ("INITIATE NETWORK...");

//CONTINUOUSLY BOARDCAST 0xFF SIGNAL IN 15S TO START INITIATE THE NETWORK
  uint32_t timeout_ms = HAL_GetTick() + 15000;
  while (HAL_GetTick() < timeout_ms)
  {
	  TxBuffer[0] = 0xFF; // Tín hiệu đồng bộ mạng
	  TxBuffer[1] = 0xFF;
	  TxBuffer[2] = 0xFF;
	  TxBuffer[3] = 0xFF;
	  LoRa_transmit(&myLoRa, TxBuffer, 4, 500);
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET); //DEBUG LED

//BOARDCAST PRE-START SIGNAL 0xC0
  TxBuffer[0] = 0xC0; //0xEE
  TxBuffer[1] = 0xEE;
  TxBuffer[2] = 0xEE;
  TxBuffer[3] = 0xEE;
  LoRa_transmit(&myLoRa, TxBuffer, 4, 500);

//WAIT UNTIL RECEIVED ALL ACK FROM SENSOR NODES
  LoRa_startReceiving(&myLoRa);
//  HAL_Delay(500); //Do we really need this?? gonna commented it to test!
  while ((receivedACK[0] != 0x01) || (receivedACK[1] != 0x02) || (receivedACK[2] != 0x03))
  {
	  //wait for ACK
  }

//LCD INDICATE THAT HAVE RECEIVED ALL ACK
  lcd_clear();
  lcd_put_cur(1, 2);
  lcd_send_string ("ALL SENSOR NODES");
  lcd_put_cur(2, 3);
  lcd_send_string ("ARE DETECTED!");
  HAL_Delay(3000);
  lcd_clear();
  lcd_put_cur(1, 2);
  lcd_send_string ("WELCOME USER TO");
  lcd_put_cur(2, 3);
  lcd_send_string (" AGRITOMATO! ");
  HAL_Delay(3000);
  lcd_clear();

//SENDING START SIGNAL 0xB0 TO ENTIRE NETWORK TO SYNCHRONIZE RTC WAKE UP OF DATA SENDIING TIMESLOT
  if ((receivedACK[0] == 0x01) && (receivedACK[1] == 0x02) && (receivedACK[2] == 0x03))
  {
	  TxBuffer[0] = 0xB0; //0xDD
	  TxBuffer[1] = 0xDD;
      TxBuffer[2] = 0xDD;
      TxBuffer[3] = 0xDD;
      LoRa_transmit(&myLoRa, TxBuffer, 4, 500);
      Start = 0;
  }
  else
  	  {

  	  }

// FINISHED SYNCHRONIZE THE NETWORK --> STEP INTO RECEIVE DATA MODE
  LoRa_startReceiving(&myLoRa);
  HAL_UART_Receive_IT(&huart1, &ESP_rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if new_feature_dev == 1

	  if (Data_Sensor_Interrupt == 1)
	  {
		  Received_Data_Handler();
		  HAL_UART_Transmit_IT(&huart1, buffer, sizeof(buffer));
//		  Sendto_ESP_UART(&sensorData);
		  LoRa_startReceiving(&myLoRa);
		  Data_Sensor_Interrupt = 0;
	  }
	  LCD_Display(&sensorData);
	  switch (control_mode)
	  {
	      case MODE_MANUAL:
	    	  if (manual_interrupt_flag == 1)
			  {
	    		  change_relay_state();
	    		  manual_interrupt_flag = 0;
			  }
	          break;

	      case MODE_AUTOMATE:
	    	  if ((RX_Data_Control[2] == 0x31) && (RX_Data_Control[3] == 0x32))
	    	  {
	    		  LoRa_transmit(&myLoRa, RX_Data_Control, sizeof(RX_Data_Control), 500);
	    		  RX_Data_Control[2] = 0;
	    		  RX_Data_Control[3] = 0;
	    		  LoRa_startReceiving(&myLoRa);
	    	  }

	    	  break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : Fan_SW_Pin Next_State_Pin Prev_State_Pin Manual_Pin
                           Light_SW_Pin Water_pump_Pin */
  GPIO_InitStruct.Pin = Fan_SW_Pin|Next_State_Pin|Prev_State_Pin|Manual_Pin
                          |Light_SW_Pin|Water_pump_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//LCD DISPLAY FUNCTION
void LCD_Display(SensorData_t *sensorData)
{
    if (Next_State_Flag)
    {
        current_display_screen = (current_display_screen + 1) % SCREEN_TOTAL;
        Next_State_Flag = 0;
        lcd_clear();
    }
    if (Prev_State_Flag)
    {
        current_display_screen = (current_display_screen + SCREEN_TOTAL - 1) % SCREEN_TOTAL;
        Prev_State_Flag = 0;
        lcd_clear();
    }
    char buffer[20];
    switch (current_display_screen)
        {
            case SCREEN_SENSOR:
                lcd_put_cur(0, 0);
                sprintf(buffer, "ID:0x%X", sensorData->Check_ID);
                lcd_send_string(buffer);

                // SOILD MOISTURE DISPLAY
                lcd_put_cur(2, 9);
                sprintf(buffer, "Mois:%d.%d%%", sensorData->soil_Moisture / 10, sensorData->soil_Moisture % 10);
                lcd_send_string(buffer);

                // TEMPERATURE DISPLAY
                lcd_put_cur(1, 0);
                sprintf(buffer, "Temp:%d.%dC", sensorData->Temperature / 10, sensorData->Temperature % 10);
                lcd_send_string(buffer);

                // LUX DISPLAY
                lcd_put_cur(2, 0);
                lcd_send_string("         ");
                lcd_put_cur(2, 0);
                sprintf(buffer, "Lux:%d.%d", sensorData->LUX / 10, sensorData->LUX % 10);
                lcd_send_string(buffer);

                // AIR MOISTURE DISPLAY
                lcd_put_cur(1, 11);
                sprintf(buffer, "Hum:%d.%d%%", sensorData->Humidity / 10, sensorData->Humidity % 10);
                lcd_send_string(buffer);

                // MQ135 DATA DISPLAY
                lcd_put_cur(3, 0);
                lcd_send_string("         ");
                lcd_put_cur(3, 0);
                sprintf(buffer, "MQ:%d", sensorData->Data_MQ135);
                lcd_send_string(buffer);

                // SOILD TEMPERATURE DISPLAY
                lcd_put_cur(3, 8);
                sprintf(buffer, "Soil:%d.%dC", sensorData->soil_Temp / 10, sensorData->soil_Temp % 10);
                lcd_send_string(buffer);

                break;
            case SCREEN_RELAY:
                lcd_put_cur(0, 3);
                lcd_send_string("RELAY STATUS");

                lcd_put_cur(1, 0);
                sprintf(buffer, "WATER: %s", water_relay ? "ON " : "OFF");
                lcd_send_string(buffer);

                lcd_put_cur(2, 0);
                sprintf(buffer, "FAN  : %s", fan_relay ? "ON " : "OFF");
                lcd_send_string(buffer);

                lcd_put_cur(3, 0);
                sprintf(buffer, "LIGHT: %s", light_relay ? "ON " : "OFF");
                lcd_send_string(buffer);
                break;

            case SCREEN_MODE:
                if (control_mode == MODE_MANUAL)
                {
                    lcd_put_cur(1, 3);
                    lcd_send_string("CURRENT MODE:");
                    lcd_put_cur(2, 3);
                    lcd_send_string("MANUAL MODE  ");
                }
                else if (control_mode == MODE_AUTOMATE)
                {
                    lcd_put_cur(1, 3);
                    lcd_send_string("CURRENT MODE:");
                    lcd_put_cur(2, 3);
                    lcd_send_string("AUTOMATE MODE");
                }
                else
                {
                    lcd_send_string("UNKNOWN");
                }
                break;

             default:

                break;
        }
}


void Received_Data_Handler(void)
{
	if(Received_Data[0] == 0xA1)
	{
		sensorData.Check_ID    = Received_Data[0];
		sensorData.Humidity    = Received_Data[2] | (Received_Data[1] << 8);
		sensorData.Temperature = Received_Data[4] | (Received_Data[3] << 8);
		sensorData.LUX         = Received_Data[6] | (Received_Data[5] << 8);
		sensorData.Data_MQ135  = Received_Data[8] | (Received_Data[7] << 8);
#if new_feature_dev == 1
		RX_Data_Control[0] = 0xA3;
		RX_Data_Control[1] = 0xBB;
		RX_Data_Control[2] = 0x31;

		buffer[0] = 0xA1;
		buffer[1] = Received_Data[1];
		buffer[2] = Received_Data[2];
		buffer[3] = Received_Data[3];
		buffer[4] = Received_Data[4];
		buffer[5] = Received_Data[5];
		buffer[6] = Received_Data[6];
		buffer[7] = Received_Data[7];
		buffer[8] = Received_Data[8];


		for (int i = 4; i < 12 ; i++) {
		    RX_Data_Control[i] = Received_Data[i-3];  // Sao chép dữ liệu cũ từ vị trí 1 trở đi
		}
		change_relay_state_update_manual();
#endif
	}

	else if(Received_Data[0] == 0xA2)
	{
		sensorData.Check_ID    = Received_Data[0];
	    sensorData.soil_Temp = Received_Data[2] | (Received_Data[1] << 8);
	    sensorData.soil_Moisture = Received_Data[4] | (Received_Data[3] << 8);
#if new_feature_dev == 1
		RX_Data_Control[3] = 0x32;

		buffer[0] = 0xA1;
		buffer[9] = Received_Data[1];
		buffer[10] = Received_Data[2];
		buffer[11] = Received_Data[3];
		buffer[12] = Received_Data[4];
		buffer[13] = (water_relay << 2) | (fan_relay << 1) | light_relay;

		for (int i = 12; i < 16 ; i++) {
		    RX_Data_Control[i] = Received_Data[i - 11];  // Sao chép dữ liệu cũ từ vị trí 1 trở đi
		}
		change_relay_state_update_manual();
#endif
	}

#if new_feature_dev == 1
    else if (Received_Data[0] == 0xA3)
    {
//    	automate_interrupt_flag = 1;
    	uint8_t relay_status = Received_Data[1];

    	light_relay  = (relay_status >> 0) & 0x01; // Lấy bit 0
    	fan_relay = (relay_status >> 1) & 0x01; // Lấy bit 1
    	water_relay = (relay_status >> 2) & 0x01; // Lấy bit 2
    }
#endif
}

// SENDING DATA TO ESP: 1 BYTE ID + 6*2 DATA BYTES + 1 BYTE RELAYS STATE = 14 BYTES. WE CAN ADD MORE 1 OR 2 HEADER BYTE IF NEEDED.
//void Sendto_ESP_UART(SensorData_t *data)
//{
//    uint8_t buffer[14];
//
//    buffer[0]  = data->Check_ID;
//
//    buffer[1]  = (uint8_t)(data->Humidity >> 8);
//    buffer[2]  = (uint8_t)(data->Humidity & 0xFF);
//
//    buffer[3]  = (uint8_t)(data->Temperature >> 8);
//    buffer[4]  = (uint8_t)(data->Temperature & 0xFF);
//
//    buffer[5]  = (uint8_t)(data->LUX >> 8);
//    buffer[6]  = (uint8_t)(data->LUX & 0xFF);
//
//    buffer[7]  = (uint8_t)(data->Data_MQ135 >> 8);
//    buffer[8]  = (uint8_t)(data->Data_MQ135 & 0xFF);
//
//    buffer[9]  = (uint8_t)(data->soil_Temp >> 8);
//    buffer[10] = (uint8_t)(data->soil_Temp & 0xFF);
//
//    buffer[11] = (uint8_t)(data->soil_Moisture >> 8);
//    buffer[12] = (uint8_t)(data->soil_Moisture & 0xFF);
//
//	// Gộp trạng thái lại thành 1 byte:
//	// Ví dụ: PA11 -> bit 2, PA10 -> bit 1, PA9 -> bit 0
//	buffer[13] = (water_relay << 2) | (fan_relay << 1) | light_relay;
//    HAL_UART_Transmit_IT(&huart1, buffer, sizeof(buffer));
//}

//THIS WAKE UP TRIGGERED BY GPIO DIO0, HAPPENS WHEN A DATA RECEIVED BY SX1278 MODULE
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO0_Pin)
    {
    	if (Start == 1) // Start over the network
    	{
    		LoRa_receive(&myLoRa, Received_Data, 9);
    		if (Received_Data[0] == 0x01)
    		{
    			receivedACK[0] = 0x01;
    		}

    		else if (Received_Data[0] == 0x02)
    		{
    			receivedACK[1] = 0x02;
    		}

    		else if (Received_Data[0] == 0x03)
    		{
    			receivedACK[2] = 0x03;
    		}
    	}
    	else
    	{
    		LoRa_receive(&myLoRa, Received_Data, 9);
    		Data_Sensor_Interrupt = 1;
    	}
    }

    else if (GPIO_Pin == Light_SW_Pin)
    {
    	control_mode = MODE_MANUAL;
//    	TxBuffer[0] = 0xAA; // Indicate to control node that the system is in manual mode
//    	LoRa_transmit(&myLoRa, TxBuffer, sizeof(TxBuffer), 500);
    	manual_interrupt_flag = 1;
    	__HAL_GPIO_EXTI_CLEAR_IT(Light_SW_Pin);  // Clear the interrupt flag
    	if (light_relay == 0)
    	{
    		//send packet to turn on light
    		light_relay = 1;
    		//remember to change the sx1278 mode to receive mode again
    	}
    	else
    	{
    		//send packet to turn off light
    		light_relay = 0;
    		//remember to change the sx1278 mode to receive mode again
    	}
    }

    else if (GPIO_Pin == Water_pump_Pin)
    {
    	control_mode = MODE_MANUAL;
//    	TxBuffer[0] = 0xBB; // Indicate to control node that the system is in manual mode
//    	LoRa_transmit(&myLoRa, TxBuffer, sizeof(TxBuffer), 500);
    	manual_interrupt_flag = 1;
    	__HAL_GPIO_EXTI_CLEAR_IT(Water_pump_Pin);  // Clear the interrupt flag
    	if (water_relay == 0)
    	{
    		//send packet to turn on pump
    		water_relay = 1;
    		//remember to change the sx1278 mode to receive mode again
    	}
    	else
    	{
    		//send packet to turn off pump
    		water_relay = 0;
    		//remember to change the sx1278 mode to receive mode again
    	}
    }

    else if (GPIO_Pin == Fan_SW_Pin)
    {
    	control_mode = MODE_MANUAL;
//    	TxBuffer[0] = 0xAA; // Indicate to control node that the system is in manual mode
//    	LoRa_transmit(&myLoRa, TxBuffer, sizeof(TxBuffer), 500);
    	manual_interrupt_flag = 1;
    	__HAL_GPIO_EXTI_CLEAR_IT(Fan_SW_Pin);  // Clear the interrupt flag
    	if (fan_relay == 0)
    	{
    		//send packet to turn on fan
    		fan_relay = 1;
    		//remember to change the sx1278 mode to receive mode again
    	}
    	else
    	{
    		//send packet to turn off fan
    		fan_relay = 0;
    		//remember to change the sx1278 mode to receive mode again
    	}
    }

    else if (GPIO_Pin == Manual_Pin)
    {
        if (control_mode == MODE_AUTOMATE)
        {
            control_mode = MODE_MANUAL;
//        	TxBuffer[0] = 0xAA; // Indicate to control node that the system is in manual mode
//        	LoRa_transmit(&myLoRa, TxBuffer, sizeof(TxBuffer), 500);
        }
        else
        {
            control_mode = MODE_AUTOMATE;
//        	TxBuffer[0] = 0xBB; // Indicate to control node that the system is in automate mode
//        	LoRa_transmit(&myLoRa, TxBuffer, sizeof(TxBuffer), 500);
        }

//        manual_interrupt_flag = 1;  // �?ặt c�? nếu bạn xử lý sau ở main
    }


    else if (GPIO_Pin == Prev_State_Pin)
    {
    	Prev_State_Flag = 1;
    }

    else if (GPIO_Pin == Next_State_Pin)
    {
    	Next_State_Flag = 1;
    }
}

//INTERRUPT WHEN HAVE A REQUEST FROM ESP32
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Xử lý dữ liệu nhận được trong rx_data (bật bơm, đèn, quạt)

    	HAL_UART_Receive_IT(&huart1, &ESP_rx_data, 1); //Cấu hình nhận data từ UART cho lần ngắt sau
		manual_interrupt_flag = 1;
		control_mode = MODE_MANUAL;
		light_relay  = (ESP_rx_data >> 0) & 0x01; // Lấy bit 0
		fan_relay = (ESP_rx_data >> 1) & 0x01; // Lấy bit 1
		water_relay = (ESP_rx_data >> 2) & 0x01; // Lấy bit 2
    }
}

void change_relay_state(void)
{
    // Prepare the packet for transmission
    relay_state = (water_relay) | (light_relay << 1) | (fan_relay << 2);
    TxBuffer[0] = 0xA3;       // Control node ID
    TxBuffer[1] = 0xAA;
    TxBuffer[2] = relay_state; // Relay state byte

    // Transmit the packet
    LoRa_transmit(&myLoRa, TxBuffer, 3, 500);

    // Switch back to receive mode
    LoRa_startReceiving(&myLoRa);
}


void change_relay_state_update_manual(void)
{
    // Prepare the packet for transmission
    relay_state = (water_relay) | (light_relay << 1) | (fan_relay << 2);
    TxBuffer[0] = 0xA3;       // Control node ID
    TxBuffer[1] = 0xCC;
    TxBuffer[2] = relay_state; // Relay state byte

    // Transmit the packet
    LoRa_transmit(&myLoRa, TxBuffer, 3, 500);

    // Switch back to receive mode
    LoRa_startReceiving(&myLoRa);
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
