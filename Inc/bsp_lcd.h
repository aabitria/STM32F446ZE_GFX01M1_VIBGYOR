/*
 * bsp_lcd.h
 *
 *  Created on: Mar 9, 2024
 *      Author: abitr
 */

#ifndef BSP_LCD_H_
#define BSP_LCD_H_

#include "reg_util.h"
#include "board.h"
#include "ili9341_reg.h"
#include "stdio.h"

#define BSP_LCD_WIDTH				240
#define BSP_LCD_HEIGHT				320


/* pixel format */
#define BSP_LCD_PIXEL_FMT_L8		1
#define BSP_LCD_PIXEL_FMT_RGB565	2
#define BSP_LCD_PIXEL_FMT_RGB666	3
#define BSP_LCD_PIXEL_FMT_RGB888	4
#define BSP_LCD_PIXEL_FMT			BSP_LCD_PIXEL_FMT_RGB565

/* orientation */
#define PORTRAIT					0
#define LANDSCAPE					1
#define BSP_LCD_ORIENTATION			LANDSCAPE

#if (BSP_LCD_ORIENTATION == PORTRAIT)
#define BSP_LCD_ACTIVE_WIDTH		BSP_LCD_WIDTH
#define BSP_LCD_ACTIVE_HEIGHT		BSP_LCD_HEIGHT
#else
#define BSP_LCD_ACTIVE_WIDTH		BSP_LCD_HEIGHT
#define BSP_LCD_ACTIVE_HEIGHT		BSP_LCD_WIDTH
#endif

#define USE_DMA 					1

typedef struct {
	uint16_t x1;
	uint16_t x2;
	uint16_t y1;
	uint16_t y2;
} lcd_area_t;

struct bsp_lcd;

typedef void (*bsp_lcd_dma_cplt_cb_t)(struct bsp_lcd *);
typedef void (*bsp_lcd_dma_err_cb_t)(struct bsp_lcd *);

typedef struct {
	uint8_t 	orientation;
	uint8_t 	pixel_format;
	uint8_t 	*draw_buffer1;
	uint8_t 	*draw_buffer2;
	uint32_t 	write_length;
	uint8_t		*buff_to_draw;
	uint8_t     *buff_to_flush;

	lcd_area_t  area;
	bsp_lcd_dma_cplt_cb_t dma_cplt_cb;
	bsp_lcd_dma_err_cb_t  dma_err_cb;
} bsp_lcd_t;


static void delay_50ms(void)
{
	for(uint32_t i = 0 ; i<(0xFFFFU * 10U);i++);
}

void bsp_lcd_init(void);
void bsp_lcd_set_background_color(uint32_t rgb888);
void bsp_lcd_fill_rect(uint32_t rgb888, uint32_t x_start, uint32_t x_width,
		               uint32_t y_start, uint32_t y_height);
#endif /* BSP_LCD_H_ */
