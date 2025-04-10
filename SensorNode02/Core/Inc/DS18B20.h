#ifndef __DS18B20_H
#define __DS18B20_H

#include "main.h" // Include các thư viện cần thiết cho hàm header này

// Định nghĩa các hằng số và biến cần thiết
#define DS18B20_PORT GPIOB
#define DS18B20_PIN GPIO_PIN_12



// Khai báo các hàm
uint8_t DS18B20_Start(void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);
void delay(uint16_t delay);
float Check_Temp(void);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
#endif /* __DS18B20_H */
