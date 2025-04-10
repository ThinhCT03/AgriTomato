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
#include "i2c-lcd.h"
#include "Lora.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
LoRa myLoRa;
uint16_t LoRa_Status;
uint8_t TxBuffer[128];
uint8_t Received_Data[9];

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
volatile uint8_t LoRa_Data_Ready = 0;
uint8_t ESP_rx_data;
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
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void LCD_Display(SensorData_t *sensorData);
void Received_Data_Handler(void);
void Sendto_ESP_UART(SensorData_t *data);
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
//  lcd_clear();
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
  LoRa_startReceiving(&myLoRa);
  HAL_UART_Receive_IT(&huart1, &ESP_rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if (LoRa_Data_Ready == 1)
//  {
//	  Received_Data_Handler();
//	  LoRa_Data_Ready = 0;
//  }
//	  LCD_Display(&sensorData);
//
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
                           Water_pump_Pin Light_SW_Pin */
  GPIO_InitStruct.Pin = Fan_SW_Pin|Next_State_Pin|Prev_State_Pin|Manual_Pin
                          |Water_pump_Pin|Light_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LCD_Display(SensorData_t *sensorData)
{
    char buffer[20];
    lcd_put_cur(0, 0);
    sprintf(buffer, "ID:0x%X", sensorData->Check_ID);
    lcd_send_string(buffer);
//
    // Hiển thị độ ẩm đất
    lcd_put_cur(2, 9);
    sprintf(buffer, "Mois:%d.%d%%", sensorData->soil_Moisture / 10, sensorData->soil_Moisture % 10);
    lcd_send_string(buffer);
//
    // Hiển thị nhiệt độ
    lcd_put_cur(1, 0);
    sprintf(buffer, "Temp:%d.%dC", sensorData->Temperature / 10, sensorData->Temperature % 10);
    lcd_send_string(buffer);
//
    // Hiển thị ánh sáng
    lcd_put_cur(2, 0);
    lcd_send_string("         "); // Clear vùng Lux
    lcd_put_cur(2, 0);
    sprintf(buffer, "Lux:%d.%d", sensorData->LUX / 10, sensorData->LUX % 10);
    lcd_send_string(buffer);
//
    // Hiển thị độ ẩm không khí
    lcd_put_cur(1, 11);
    sprintf(buffer, "Hum:%d.%d%%", sensorData->Humidity / 10, sensorData->Humidity % 10);
    lcd_send_string(buffer);
//
    // Hiển thị MQ135
    lcd_put_cur(3, 0);
    lcd_send_string("         "); // Clear vùng MQ
    lcd_put_cur(3, 0);
    sprintf(buffer, "MQ:%d", sensorData->Data_MQ135);
    lcd_send_string(buffer);
//
    // Hiển thị nhiệt độ đất
    lcd_put_cur(3, 8);
    sprintf(buffer, "Soil:%d.%dC", sensorData->soil_Temp / 10, sensorData->soil_Temp % 10);
    lcd_send_string(buffer);
}

void Received_Data_Handler(void)
{
	//Received_Data đang có 10 byte, compress và chia lại thành data có ý nghĩa + gửi ACK nếu check ID đúng
	if(Received_Data[0] == 0xA1)
	{
		TxBuffer[0] = 0x01; //ACK Data send back from Gateway to Sensor node
		LoRa_transmit(&myLoRa, TxBuffer, 1, 500);
		sensorData.Check_ID    = Received_Data[0];
		sensorData.Humidity    = Received_Data[2] | (Received_Data[1] << 8);
		sensorData.Temperature = Received_Data[4] | (Received_Data[3] << 8);
		sensorData.LUX         = Received_Data[6] | (Received_Data[5] << 8);
		sensorData.Data_MQ135  = Received_Data[8] | (Received_Data[7] << 8);
	}

	else if(Received_Data[0] == 0xA2)
	{
		TxBuffer[0] = 0x02; //ACK Data send back from Gateway to Sensor node
		LoRa_transmit(&myLoRa, TxBuffer, 1, 500);
		sensorData.Check_ID    = Received_Data[0];
	    sensorData.soil_Temp = Received_Data[2] | (Received_Data[1] << 8);
	    sensorData.soil_Moisture = Received_Data[4] | (Received_Data[3] << 8);
	}
}

void Sendto_ESP_UART(SensorData_t *data)
{
    uint8_t buffer[14]; // 1 byte ID + 6*2 bytes = 13 bytes. Có thể thêm 1-2 byte header nếu cần.

    buffer[0]  = data->Check_ID;

    buffer[1]  = (uint8_t)(data->Humidity >> 8);
    buffer[2]  = (uint8_t)(data->Humidity & 0xFF);

    buffer[3]  = (uint8_t)(data->Temperature >> 8);
    buffer[4]  = (uint8_t)(data->Temperature & 0xFF);

    buffer[5]  = (uint8_t)(data->LUX >> 8);
    buffer[6]  = (uint8_t)(data->LUX & 0xFF);

    buffer[7]  = (uint8_t)(data->Data_MQ135 >> 8);
    buffer[8]  = (uint8_t)(data->Data_MQ135 & 0xFF);

    buffer[9]  = (uint8_t)(data->soil_Temp >> 8);
    buffer[10] = (uint8_t)(data->soil_Temp & 0xFF);

    buffer[11] = (uint8_t)(data->soil_Moisture >> 8);
    buffer[12] = (uint8_t)(data->soil_Moisture & 0xFF);

    // Byte còn lại là trạng thái của 3 Relay tại node control
    buffer[13] = 0x0A;

    HAL_UART_Transmit_IT(&huart1, buffer, sizeof(buffer));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Sự kiện callback xảy ra khi có data đến
{
    if (GPIO_Pin == DIO0_Pin)  // Có data đến SX1278 LoRA
    {
//      LoRa_Data_Ready = 1;
  	  LoRa_receive(&myLoRa, Received_Data, 9);
  	  Received_Data_Handler();
	  LCD_Display(&sensorData);
	  Sendto_ESP_UART(&sensorData);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // Kiểm tra đúng UART1
    {
        // Xử lý dữ liệu nhận được trong rx_data (bật bơm, đèn, quạt)

    	  HAL_UART_Receive_IT(&huart1, &ESP_rx_data, 1);; //Cấu hình nhận data từ UART cho lần ngắt sau
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
