#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change to hi2c2 if you used I2C2

#define SLAVE_ADDRESS_LCD 0x4E // Default for many PCF8574. Try 0x7E or 0x27 if this fails.

void LCD_SendCmd (char cmd)
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
}

void LCD_SendData (char data)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data&0xf0);
  data_l = ((data<<4)&0xf0);
  data_t[0] = data_u|0x0D;  //en=1, rs=1
  data_t[1] = data_u|0x09;  //en=0, rs=1
  data_t[2] = data_l|0x0D;  //en=1, rs=1
  data_t[3] = data_l|0x09;  //en=0, rs=1
  HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void LCD_Init (void)
{
  LCD_SendCmd (0x33);
  LCD_SendCmd (0x32);
  LCD_SendCmd (0x28); // Function set: DL=4-bit, N=2 line
  LCD_SendCmd (0x0C); // Display on, Cursor off
  LCD_SendCmd (0x06); // Entry mode: Auto increment
  LCD_SendCmd (0x01); // Clear display
  HAL_Delay(2);
}

void LCD_SendString (char *str)
{
  while (*str) LCD_SendData (*str++);
}

void LCD_Clear (void)
{
  LCD_SendCmd(0x01);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t pos;
    if (row == 0) pos = 0x80 + col;
    else pos = 0xC0 + col;
    LCD_SendCmd(pos);
}
