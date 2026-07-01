#ifndef SSD1306_SAFE_H
#define SSD1306_SAFE_H

#include "main.h"
#include <stdint.h>

#define SSD1306_WIDTH             128U
#define SSD1306_HEIGHT             64U
#define SSD1306_PAGES               8U

/* Use 2 for many SH1106 1.3 inch modules if the image is shifted. */
#define SSD1306_COLUMN_OFFSET       0U

uint8_t SSD1306_Init(I2C_HandleTypeDef *hi2c);
uint8_t SSD1306_IsReady(void);
uint8_t SSD1306_Update(void);
uint8_t SSD1306_UpdatePage(uint8_t page);
void SSD1306_ClearBuffer(void);
void SSD1306_Clear(void);
void SSD1306_ClearLine(uint8_t page);
void SSD1306_Print(uint8_t page, uint8_t column, const char *text);
void SSD1306_PrintNoUpdate(uint8_t page, uint8_t column, const char *text);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);

/* Compatibility names for older code. */
uint8_t OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_Print(uint8_t page, uint8_t column, const char *text);

#endif
