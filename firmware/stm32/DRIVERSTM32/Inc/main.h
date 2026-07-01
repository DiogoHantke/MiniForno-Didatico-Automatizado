#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Legacy aliases kept so older CubeMX-generated stm32f4xx_hal_msp.c still compiles. */
#define LM35_Pin GPIO_PIN_0
#define LM35_GPIO_Port GPIOA

/* DHT11 data pin. */
#define DHT11_Pin GPIO_PIN_0
#define DHT11_GPIO_Port GPIOA

#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

#define ESP_TX_Pin GPIO_PIN_9
#define ESP_TX_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_10
#define ESP_RX_GPIO_Port GPIOA

#define HEATER_PWM_Pin GPIO_PIN_0
#define HEATER_PWM_GPIO_Port GPIOB

#define LED_EMERGENCY_Pin GPIO_PIN_14
#define LED_EMERGENCY_GPIO_Port GPIOC
#define LED_READY_Pin GPIO_PIN_15
#define LED_READY_GPIO_Port GPIOC
#define LED_HEATING_Pin GPIO_PIN_1
#define LED_HEATING_GPIO_Port GPIOB
#define LED_HOLDING_Pin GPIO_PIN_2
#define LED_HOLDING_GPIO_Port GPIOB
#define LED_DONE_Pin GPIO_PIN_3
#define LED_DONE_GPIO_Port GPIOB
#define LED_PWM_MIRROR_Pin GPIO_PIN_4
#define LED_PWM_MIRROR_GPIO_Port GPIOB

#define BTN_PLUS_Pin GPIO_PIN_12
#define BTN_PLUS_GPIO_Port GPIOB
#define BTN_MINUS_Pin GPIO_PIN_13
#define BTN_MINUS_GPIO_Port GPIOB
#define BTN_ENTER_Pin GPIO_PIN_14
#define BTN_ENTER_GPIO_Port GPIOB

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
