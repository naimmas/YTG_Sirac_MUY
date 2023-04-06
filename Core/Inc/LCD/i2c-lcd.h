#include "stm32h7xx_hal.h"

typedef struct lcd_t
{
	I2C_HandleTypeDef* i2c;
	uint8_t addrs;
}LcdDev;

extern LcdDev lcd1;

void lcd_init (LcdDev* lcddev);  // initialize lcd

void lcd_send_cmd (char cmd, LcdDev* lcddev);  // send command to the lcd

void lcd_send_data (char data, LcdDev* lcddev);  // send data to the lcd

void lcd_send_string(uint8_t row, uint8_t col, char *str, LcdDev* lcddev);  // send string to the lcd

//void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (LcdDev* lcddev);
