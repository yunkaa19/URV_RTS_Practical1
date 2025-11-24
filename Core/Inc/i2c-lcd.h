#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_

#include "stm32f7xx_hal.h" // Keep your F7 header

// Initialize the screen
void LCD_Init(void);

// Print text
void LCD_SendString(char *str);

// Move cursor (Row 0 or 1, Col 0 to 15)
void LCD_SetCursor(uint8_t row, uint8_t col);

// Clear screen
void LCD_Clear(void);

// Set Backlight Color (Red, Green, Blue: 0-255)
void LCD_SetRGB(uint8_t r, uint8_t g, uint8_t b);

#endif /* INC_I2C_LCD_H_ */
