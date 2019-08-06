#include "vamk_lcd.h"

void LCD_Init(I2C_HandleTypeDef i2c_handler)
{
	/*
	Datasheet for commands/instructions
	https://panda-bg.com/datasheet/2134-091834-LCD-module-TC1602D-02WA0-16x2-STN.pdf
	https://www.electronicwings.com/sensors-modules/lcd-16x2-display-module
	https://mil.ufl.edu/3744/docs/lcdmanual/commands.html
	https://www.robofun.ro/docs/rc1602b-biw-esx.pdf
	*/
	LCD_SendCommand(0x02, i2c_handler);	// set the LCD in 4-bit mode (D4-D7)
	LCD_SendCommand(0x28, i2c_handler);	// 2 lines, 5x8 matrix, 4-bit mode
	LCD_SendCommand(0x0C, i2c_handler);	// Display ON, cursor off
	LCD_SendCommand(0x80, i2c_handler);	// Force the cursor to position (0,0)
}

void LCD_SendChar(uint8_t c, I2C_HandleTypeDef i2c_handler)
{
	uint8_t nibble_r, nibble_l;
	uint8_t data[4];
	nibble_l = c & 0xf0;
	nibble_r = (c << 4) & 0xf0;
	data[0] = nibble_l | 0x0D;
	data[1] = nibble_l | 0x09;
	data[2] = nibble_r | 0x0D;
	data[3] = nibble_r | 0x09;
	HAL_I2C_Master_Transmit(&i2c_handler, LCD_ADDR, (uint8_t *)data, 4, 100);
}
void LCD_SendCommand(uint8_t cmd, I2C_HandleTypeDef i2c_handler)
{
	uint8_t nibble_r, nibble_l;
	uint8_t data[4];
	nibble_l = cmd & 0xf0;
	nibble_r = (cmd << 4) & 0xf0;
	data[0] = nibble_l | 0x0C;
	data[1] = nibble_l | 0x08;
	data[2] = nibble_r | 0x0C;
	data[3] = nibble_r | 0x08;
	HAL_I2C_Master_Transmit(&i2c_handler, LCD_ADDR, (uint8_t *)data, 4, 100);
}

void LCD_SendString(uint8_t *str, I2C_HandleTypeDef i2c_handler)
{
	while (*str)
		LCD_SendChar(*str++, i2c_handler);
}
void DisplayOnLCD_Temperature(int temp, I2C_HandleTypeDef i2c_handler, uint8_t data_line1[], uint8_t data_line2[])
{

	sprintf((char *)data_line2, "%d C", temp);
	LCD_ClearFromPos(0,1, i2c_handler);
	LCD_GoToXY(0, 1, i2c_handler);
	LCD_SendString(data_line1, i2c_handler);
	LCD_GoToXY(0, 2, i2c_handler);
	LCD_SendString(data_line2, i2c_handler);
}
void LCD_Display(I2C_HandleTypeDef i2c_handler, uint8_t line, uint8_t * data)
{
	if(line == 1)
	{
		LCD_ClearLine(i2c_handler, 1);
		LCD_GoToXY(0, 1, i2c_handler);
		LCD_SendString(data, i2c_handler);
	}

	else if(line == 2)
	{
		LCD_ClearLine(i2c_handler, 2);
		LCD_GoToXY(0, 2, i2c_handler);
		LCD_SendString(data, i2c_handler);
	}
}
void LCD_GoToXY(uint8_t x, uint8_t y, I2C_HandleTypeDef i2c_handler)
{
	uint8_t LCD_DDRAM_ADDR = 0x80;

	if (y == 1)
		LCD_SendCommand(LCD_DDRAM_ADDR | (LCD_LINE_1 + x), i2c_handler);
	else if (y == 2)
		LCD_SendCommand(LCD_DDRAM_ADDR | (LCD_LINE_2 + x), i2c_handler);
}

void LCD_ClearAll(I2C_HandleTypeDef i2c_handler)
{
	LCD_SendCommand(0x01, i2c_handler); // 0x01 is the command to clear the LCD display
}
void LCD_ClearFromPos(uint8_t x, uint8_t y, I2C_HandleTypeDef i2c_handler)
{
	uint8_t str[33] = "                                ";
	LCD_GoToXY(x, y, i2c_handler);
	LCD_SendString(str, i2c_handler);
}
void LCD_ClearLine(I2C_HandleTypeDef i2c_handler, uint8_t line)
{
	uint8_t str[17] = "                ";
	if (line == 1) LCD_GoToXY(0, 1, i2c_handler);
	else if (line == 2) LCD_GoToXY(0, 2, i2c_handler);
	LCD_SendString(str, i2c_handler);
}
