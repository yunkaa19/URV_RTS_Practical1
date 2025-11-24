#include "i2c-lcd.h"

extern I2C_HandleTypeDef hi2c1; // Make sure this matches your I2C handle

// Grove RGB LCD Addresses (Shifted left by 1 for STM32 HAL)
#define LCD_ADDRESS     (0x3E << 1) // 0x7C
#define RGB_ADDRESS     (0x62 << 1) // 0xC4

void LCD_SendCommand(uint8_t cmd) {
    uint8_t data[2] = {0x80, cmd}; // 0x80 = Control Byte (Command)
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, data, 2, 100);
}

void LCD_SendData(uint8_t data) {
    uint8_t buffer[2] = {0x40, data}; // 0x40 = Control Byte (Data)
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, buffer, 2, 100);
}

void LCD_Init(void) {
    HAL_Delay(50);

    // --- Text Initialization ---
    LCD_SendCommand(0x28); // 2 lines, 5x8 dots
    LCD_SendCommand(0x0C); // Display ON, Cursor OFF
    LCD_SendCommand(0x06); // Auto Increment
    LCD_SendCommand(0x01); // Clear Display
    HAL_Delay(10);

    // --- RGB Backlight Initialization ---
    // Reset the RGB chip
    // --- RGB Backlight Initialization (Robust Version) ---

        // 1. Reset the RGB chip (Register 0x00 -> 0x00)
        uint8_t reg0[2] = {0x00, 0x00};
        HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, reg0, 2, 100);

        // 2. Enable LEDs (Register 0x08 -> 0xAA) <--- CRITICAL for some Clones
        uint8_t regOutput[2] = {0x08, 0xAA};
        HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, regOutput, 2, 100);

        // 3. Set standard color (White)
        LCD_SetRGB(255, 255, 255);
}

void LCD_SetRGB(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t data[2];

    // Red
    data[0] = 0x04; data[1] = r;
    HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, data, 2, 100);

    // Green
    data[0] = 0x03; data[1] = g;
    HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, data, 2, 100);

    // Blue
    data[0] = 0x02; data[1] = b;
    HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, data, 2, 100);
}

void LCD_SendString(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t cmd = (row == 0 ? 0x80 : 0xC0) + col;
    LCD_SendCommand(cmd);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}
