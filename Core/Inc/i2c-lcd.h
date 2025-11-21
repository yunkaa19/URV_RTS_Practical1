#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_

#include "stm32f7xx_hal.h" // Change to f4xx if using F4, but you have F767ZI

void LCD_Init (void);   // initialize lcd
void LCD_SendString (char *str);  // send string to the lcd
void LCD_PutChar (char c);  // send char to the lcd
void LCD_Clear (void);  // clear lcd
void LCD_SetCursor(uint8_t row, uint8_t col); // set cursor position

#endif /* INC_I2C_LCD_H_ */