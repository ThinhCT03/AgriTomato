#include "DS18B20.h"


extern TIM_HandleTypeDef htim1;

uint8_t DS18B20_Start(void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
    delay (80);    // delay according to datasheet
    if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
    else Response = -1;

    delay (400); // 480 us delay totally.

    return Response;
}

void DS18B20_Write(uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Set as output

	for (int i = 0; i < 8; i++)
	{
		if ((data & (1 << i)) != 0)  // Nếu bit = 1
		{
			// Write 1
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // Kéo xuống LOW
			delay(1);
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Nhả chân (Input để pull-up)
			delay(50);  // Đợi đủ 60µs
		}
		else  // Nếu bit = 0
		{
			// Write 0
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // Giữ LOW
			delay(50);  // Chờ đủ 60µs
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Nhả chân
		}
	}
}


uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0; i<8; i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // Set as output
		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);   // Pull data pin LOW
		delay (2);  // Chờ ít nhất 1µs
		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);   // Thả chân, chuyển sang Input

		delay(15);
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // Nếu chân ở mức HIGH
		{
			value |= 1 << i;  // Ghi nhận bit = 1
		}

		delay (45);  // Hoàn tất read slot (tổng 60µs)
	}
	return value;
}


void delay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

float Check_Temp(void)
{
	  uint8_t Temp_byte1, Temp_byte2;
	  uint8_t Presence;
	  uint16_t TEMP;
	  float Temperature;
	    Presence = DS18B20_Start();
	    HAL_Delay (1);
	    if(Presence == 1){
	    DS18B20_Write (0xCC);  // skip ROM
	    DS18B20_Write (0x44);  // convert t
	    }

	    HAL_Delay(800);
	    Presence = DS18B20_Start();
	    HAL_Delay (1);
	    if(Presence == 1){
	    DS18B20_Write (0xCC);  // skip ROM
	    DS18B20_Write (0xBE);  // Read Scratch-pad
	    }

	  Temp_byte1 = DS18B20_Read();
	  Temp_byte2 = DS18B20_Read();
	  TEMP = (Temp_byte2<<8)|Temp_byte1;
	  Temperature = (float)TEMP/16;
	  return Temperature;
}

