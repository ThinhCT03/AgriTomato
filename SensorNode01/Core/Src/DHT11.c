#include "DHT11.h"


extern TIM_HandleTypeDef htim1;

void DHT11_Start (void) // THIS FUNCTION WILL SEND THE HIGH START SIGNAL FORM MCU TO DHT11
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	delay (18000);   // wait 18ms FOR DHTT11 TO PREPAIR THE DATA
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}


uint8_t DHT11_Read (void) // READ THE DATA; WAIT 40US IF THE STATE IS LOW THEN THE BIT IS "0" ELSE WILL "1". BECAUSE HIGH SIGNAL WITHIN 26-28US IS "0" AND 70US IS "1".
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}
uint8_t DHT11_Check_Response (void) // CHECK THE RESPOND ACK FROM THE DHT11: 40us pull up by MCU, 80us pull down and 80us pull up ACK signals by DHT11
{
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		delay (90);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1;
	}
	if (Response == 1) {
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // if received the ACK, we wait for the DHT11 pull the pin to go low
	}

	return Response;
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

float Check_RH(void)
{
	  uint8_t Rh_byte1, Rh_byte2;
	  float Humidity;
	  Rh_byte1 = DHT11_Read();
	  Rh_byte2 = DHT11_Read();
	  Humidity = (float)(Rh_byte1 + Rh_byte2 / 10.0);
	  return Humidity;
}

float Check_Temp(void)
{
	  uint8_t Temp_byte1, Temp_byte2;
	  float Temperature;
	  Temp_byte1 = DHT11_Read();
	  Temp_byte2 = DHT11_Read();
	  Temperature = (float)(Temp_byte1 + Temp_byte2 / 10.0);
	  return Temperature;
}

//void Display_Temp (float Temp)
//{
//    char str[20] = {0};
//    int integerPart = (int)Temp;
//    int decimalPart = (int)((Temp - integerPart) * 10);
//    int index = 0;
//
//    lcd_put_cur(0, 0);
//
//    // Add "TE:" prefix
//    str[index++] = 'T';
//    str[index++] = 'E';
//    str[index++] = ':';
//
//    // Add integer part
//    if (integerPart == 0) {
//        str[index++] = '0';
//    } else {
//        if (integerPart >= 10) {
//            str[index++] = '0' + (integerPart / 10);
//        }
//        str[index++] = '0' + (integerPart % 10);
//    }
//
//    // Add decimal point
//    str[index++] = '.';
//
//    // Add decimal part
//    str[index++] = '0' + decimalPart;
//
//    // Null-terminate the string
//    str[index] = '\0';
//
//    lcd_send_string(str);
//    lcd_send_data('C');
//}
//
//
//void Display_Rh (float Rh)
//{
//    char str[20] = {0};
//    int integerPart = (int)Rh;
//    int decimalPart = (int)((Rh - integerPart) * 10);
//    int index = 0;
//
//    lcd_put_cur(1, 0);
//
//    // Add "HU:" prefix
//    str[index++] = 'H';
//    str[index++] = 'U';
//    str[index++] = ':';
//
//    // Add integer part
//    if (integerPart == 0) {
//        str[index++] = '0';
//    } else {
//        if (integerPart >= 10) {
//            str[index++] = '0' + (integerPart / 10);
//        }
//        str[index++] = '0' + (integerPart % 10);
//    }
//
//    // Add decimal point
//    str[index++] = '.';
//
//    // Add decimal part
//    str[index++] = '0' + decimalPart;
//
//    // Null-terminate the string
//    str[index] = '\0';
//
//    lcd_send_string(str);
//    lcd_send_data('%');
//}
