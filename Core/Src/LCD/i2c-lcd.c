/** Put this in the src folder **/

#include "LCD/i2c-lcd.h"

#define SLAVE_ADDRESS_LCD 0x38<<1 // change this according to ur setup

void lcd_send_cmd(char cmd, LcdDev* lcddev)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C;  //en=1, rs=0
	data_t[1] = data_u | 0x08;  //en=0, rs=0
	data_t[2] = data_l | 0x0C;  //en=1, rs=0
	data_t[3] = data_l | 0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit(lcddev->i2c, lcddev->addrs, (uint8_t*) data_t, 4,
			100);
}

void lcd_send_data(char data, LcdDev* lcddev)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D;  //en=1, rs=0
	data_t[1] = data_u | 0x09;  //en=0, rs=0
	data_t[2] = data_l | 0x0D;  //en=1, rs=0
	data_t[3] = data_l | 0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit(lcddev->i2c, lcddev->addrs, (uint8_t*) data_t, 4,
			100);
}

void lcd_clear(LcdDev* lcddev)
{
	lcd_send_cmd(0x80, lcddev);
	for (int i = 0; i < 70; i++)
	{
		lcd_send_data(' ', lcddev);
	}
	HAL_Delay(25);
}
/*

void lcd_put_cur(int row, int col)
{
	switch (row)
	{
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}

	lcd_send_cmd(col);
	HAL_Delay(5);
}
*/

void lcd_init(LcdDev* lcddev)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd(0x30, lcddev);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd(0x30, lcddev);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd(0x30, lcddev);
	HAL_Delay(10);
	lcd_send_cmd(0x20, lcddev);  // 4bit mode
	HAL_Delay(10);

	// dislay initialisation
	lcd_send_cmd(0x28, lcddev); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd(0x08, lcddev); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd(0x01, lcddev);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd(0x06, lcddev); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd(0x0C, lcddev); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string(uint8_t row, uint8_t col, char *str, LcdDev* lcddev)
{
	switch (row)
	{
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}

	lcd_send_cmd(col, lcddev);
	HAL_Delay(5);
	while (*str)
		lcd_send_data(*str++, lcddev);
	HAL_Delay(20);
}
