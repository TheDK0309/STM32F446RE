#ifndef VAMK_LCD_H_
#define VAMK_LCD_H_



#endif /* VAMK_LCD_H_ */
#include "main.h"
#include "stm32l1xx_hal.h"

#define LCD_ADDR 0x4E
#define LCD_LINE_1 0x00
#define LCD_LINE_2 0x40
/* LCD prototypes  */
void DisplayOnLCD_Temperature(float temp, I2C_HandleTypeDef i2c_handler, uint8_t data_line1[], uint8_t data_line2[]);
void LCD_Init(I2C_HandleTypeDef i2c_handler);
void LCD_SendCommand(uint8_t cmd, I2C_HandleTypeDef i2c_handler);
void LCD_SendChar(uint8_t c, I2C_HandleTypeDef i2c_handler);
void LCD_SendString(uint8_t *str, I2C_HandleTypeDef i2c_handler);
void LCD_GoToXY(uint8_t x, uint8_t y, I2C_HandleTypeDef i2c_handler);
void LCD_ClearAll(I2C_HandleTypeDef i2c_handler);
void LCD_ClearFromPos(uint8_t x, uint8_t y, I2C_HandleTypeDef i2c_handler);
void LCD_Display(I2C_HandleTypeDef i2c_handler, uint8_t line, uint8_t * data);
void LCD_ClearLine(I2C_HandleTypeDef i2c_handler, uint8_t line);
