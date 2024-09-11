/*
 * board.h
 *
 *  Created on: Aug 18, 2024
 *      Author: abitr
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "stm32f446xx.h"

#define MCU_GPIO_PIN_0					0U
#define MCU_GPIO_PIN_1					1U
#define MCU_GPIO_PIN_2					2U
#define MCU_GPIO_PIN_3					3U
#define MCU_GPIO_PIN_4					4U
#define MCU_GPIO_PIN_5					5U
#define MCU_GPIO_PIN_6					6U
#define MCU_GPIO_PIN_7					7U
#define MCU_GPIO_PIN_8					8U
#define MCU_GPIO_PIN_9					9U
#define MCU_GPIO_PIN_10					10U
#define MCU_GPIO_PIN_11					11U
#define MCU_GPIO_PIN_12					12U
#define MCU_GPIO_PIN_13					13U
#define MCU_GPIO_PIN_14					14U
#define MCU_GPIO_PIN_15					15U

#define SPI								SPI1
#define LCD_SCL_PIN						MCU_GPIO_PIN_5
#define LCD_SCL_PORT					GPIOA
#define LCD_SDI_PIN						MCU_GPIO_PIN_6
#define LCD_SDI_PORT					GPIOA
#define LCD_SDO_PIN						MCU_GPIO_PIN_7
#define LCD_SDO_PORT					GPIOA
#define LCD_RESX_PIN					MCU_GPIO_PIN_1
#define LCD_RESX_PORT					GPIOA
#define LCD_CSX_PIN						MCU_GPIO_PIN_5
#define LCD_CSX_PORT					GPIOB
#define LCD_DCX_PIN						MCU_GPIO_PIN_3
#define LCD_DCX_PORT					GPIOB



#endif /* BOARD_H_ */
