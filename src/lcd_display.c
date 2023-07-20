/*
 * lcd_display.c
 *
 *  Created on: 20.07.2023
 *      Author: Admin
 */
#include "lcd_display.h"

void lcdSendComd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	HAL_Delay(300);
}

void lcdSendData (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcdClear (void)
{
	lcdSendComd (0x80);
	for (int i=0; i<70; i++)
	{
		lcdSendData (' ');
	}
}

void lcdSetCursor(int row, int col)
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

    lcdSendComd (col);
}


void lcdInit (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcdSendComd (0x30);
	HAL_Delay(50);  // wait for >4.1ms
	lcdSendComd (0x30);
	HAL_Delay(10);  // wait for >100us
	lcdSendComd (0x30);
	HAL_Delay(10);
	lcdSendComd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcdSendComd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(10);
	lcdSendComd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(10);
	lcdSendComd (0x01);  // clear display
	HAL_Delay(10);
	HAL_Delay(10);
	lcdSendComd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(10);
	lcdSendComd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcdSendString (char *str)
{
	while (*str) lcdSendData (*str++);
}

void updateLCD(char *str)
{
	lcdClear();
	HAL_Delay(50);
	lcdSetCursor(0,0);
	HAL_Delay(50);
	lcdSendString(str);
}
