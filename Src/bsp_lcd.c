/*
 * bsp_lcd.c
 *
 *  Created on: Mar 9, 2024
 *      Author: abitr
 */
#include "bsp_lcd.h"
#include "arm_cm4.h"

#define __enable_irq()				do{\
										uint32_t priMask = 0;\
										__asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");}while(0)

#define __disable_irq()				do{\
										uint32_t priMask = 1;\
										__asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");}while(0)

#define __spi_set_dff_8bit()  		REG_CLR_BIT(SPI->CR1, SPI_CR1_DFF_Pos)
#define __spi_set_dff_16bit()		REG_SET_BIT(SPI->CR1, SPI_CR1_DFF_Pos)
#define __disable_spi()				REG_CLR_BIT(SPI->CR1, SPI_CR1_SPE_Pos)
#define __enable_spi()				REG_SET_BIT(SPI->CR1, SPI_CR1_SPE_Pos)

#define __enable_dma(stream)        REG_SET_BIT(stream->CR, DMA_SxCR_EN_Pos);
#define __disable_dma(stream)       REG_CLR_BIT(stream->CR, DMA_SxCR_EN_Pos);

#define LCD_RESX_HIGH()				REG_SET_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)
#define LCD_RESX_LOW()				REG_CLR_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)

#define LCD_CSX_HIGH()				REG_SET_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)
#define LCD_CSX_LOW()				REG_CLR_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)

#define LCD_DCX_HIGH()				REG_SET_BIT(LCD_DCX_PORT->ODR, LCD_DCX_PIN)
#define LCD_DCX_LOW()				REG_CLR_BIT(LCD_DCX_PORT->ODR, LCD_DCX_PIN)

#define MADCTL_MY 					0x80  ///< Bottom to top
#define MADCTL_MX 					0x40  ///< Right to left
#define MADCTL_MV 					0x20  ///< Reverse Mode
#define MADCTL_ML 					0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 					0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 					0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 					0x04  ///< LCD refresh right to left

#define DB_SIZE						(10UL * 1024UL)

uint8_t bsp_db[DB_SIZE];
uint8_t bsp_wb[DB_SIZE];

bsp_lcd_t lcd_handle = {0};
bsp_lcd_t *hlcd = &lcd_handle;


static void dma_lcd_write_error (bsp_lcd_t *lcd)
{
	while(1);
}

static void dma_cmplt_callback_spi_write(bsp_lcd_t *lcd)
{
	// this is what's being waited to be acknowledged in the main thrd
	lcd->buff_to_flush = NULL;

#if (USE_DMA == 1)
	LCD_CSX_HIGH();
	__disable_spi();
	__spi_set_dff_8bit();
	__enable_spi();
#endif
}

void dma_copy_m2p (uint32_t src_addr, uint32_t dst_addr, uint32_t nitems)
{
	DMA_Stream_TypeDef *pStream = DMA2_Stream3;

	__disable_dma(pStream);

	// Configure addresses
	REG_SET_VAL(pStream->PAR, dst_addr, 0xFFFFFFFFUL, DMA_SxPAR_PA_Pos);
	REG_SET_VAL(pStream->M0AR, src_addr, 0xFFFFFFFFUL, DMA_SxM0AR_M0A_Pos);

	// Set transfer length
	REG_SET_VAL(pStream->NDTR, nitems, 0xFFFFU, DMA_SxNDT_Pos);

	__enable_dma(pStream);

	// Check if we can place it in one-time call init fns
	REG_SET_BIT(SPI->CR2, SPI_CR2_TXDMAEN_Pos);
}

void lcd_write_dma(uint32_t src_addr, uint32_t nbytes)
{
	uint32_t nitems;

	__disable_spi();
	__spi_set_dff_16bit();
	__enable_spi();
	LCD_CSX_LOW();

	nitems = nbytes /2;
	dma_copy_m2p((uint32_t)src_addr,(uint32_t)&SPI->DR, nitems);
}

// DMA channel and streams for SPI1:
// SPI1_RX DMA2, Ch3, Str0/2
// SPI1_TX DMA2, Ch3, Str3/5
void initialize_lcd_write_dma(uint32_t src_addr, uint32_t dst_addr)
{
	DMA_TypeDef *pDMA = DMA2;
	RCC_TypeDef *pRCC = RCC;
	DMA_Stream_TypeDef *pStream = DMA2_Stream3;

	//Enable clock for DMA1
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_Pos);

	/*Stream configuration */
	REG_CLR_BIT(pStream->CR, DMA_SxCR_EN_Pos); /*Make sure that stream is disabled */
	REG_SET_VAL(pStream->CR, 0x3, 0x7U, DMA_SxCR_CHSEL_Pos); /* SPI1_TX is on channel 3 */
	REG_SET_VAL(pStream->CR, 0x00, 0x3U, DMA_SxCR_MBURST_Pos); /*Single transfer*/
	REG_SET_VAL(pStream->CR, 0x3U, 0x3U, DMA_SxCR_PL_Pos); /*Priority very high*/
	REG_CLR_BIT(pStream->CR, DMA_SxCR_PINCOS_Pos);
	REG_SET_VAL(pStream->CR, 0x1U, 0x3U, DMA_SxCR_MSIZE_Pos); /* MSIZE = 16b */
	REG_SET_VAL(pStream->CR, 0x1U, 0x3U, DMA_SxCR_PSIZE_Pos); /* PSIZE = 16b */
	REG_SET_BIT(pStream->CR, DMA_SxCR_MINC_Pos); /* Increment memory address*/
	REG_CLR_BIT(pStream->CR, DMA_SxCR_PINC_Pos); /* Fixed peripheral address */
	REG_SET_VAL(pStream->CR, 0x1U, 0x3U, DMA_SxCR_DIR_Pos); /* Direction : Memory to peripheral */
	REG_CLR_BIT(pStream->CR, DMA_SxCR_PFCTRL_Pos); /* Flow controller = DMA */

	/*Address configuration */
	REG_SET_VAL(pStream->PAR, dst_addr, 0xFFFFFFFFU, DMA_SxPAR_PA_Pos);
	REG_SET_VAL(pStream->M0AR, src_addr, 0xFFFFFFFFU, DMA_SxM0AR_M0A_Pos);

	/*FIFO control*/
	REG_CLR_BIT(pStream->FCR,DMA_SxFCR_DMDIS_Pos); /* Direct mode enabled */


	/*Stream interrupt configuration */
	REG_SET_BIT(pStream->CR,DMA_SxCR_TCIE_Pos); /*Enable Transfer complete interrupt */
	REG_SET_BIT(pStream->CR,DMA_SxCR_TEIE_Pos); /* Enable transfer error interrupt */
	REG_SET_BIT(pStream->CR,DMA_SxCR_DMEIE_Pos); /* Enable direct mode error interrupt */

	/*Enable IRQ for stream4 in Nvic */
	REG_SET_BIT(NVIC_ISER1, (DMA2_Stream3_IRQn % 32));
}

void lcd_dma_init(bsp_lcd_t *lcd)
{
#if (USE_DMA == 1)
	initialize_lcd_write_dma((uint32_t)bsp_wb, (uint32_t)&SPI->DR);
#endif
}

void lcd_write_cmd(uint8_t cmd)
{
    SPI_TypeDef *pSPI = SPI;
    uint8_t data;

    LCD_CSX_LOW();
    LCD_DCX_LOW();  // DCX=0 is for cmd

    while(!REG_READ_BIT(pSPI->SR, SPI_SR_TXE_Pos)); // wait for TX buffer to empty
    REG_WRITE(pSPI->DR, cmd);
    while(!REG_READ_BIT(pSPI->SR, SPI_SR_TXE_Pos)); // wait for cmd to be sent
    while(REG_READ_BIT(pSPI->SR, SPI_SR_BSY_Pos)); // wait for the SPI session to finish
    data = REG_READ(pSPI->DR) & 0xFF;

    LCD_DCX_HIGH();  // DCX=0 is for cmd
    LCD_CSX_HIGH();
}

#if 1
void lcd_write_data(uint8_t *buffer, uint32_t len)
{
    SPI_TypeDef *pSPI = SPI;
    uint8_t data;

    LCD_CSX_LOW();

    for (uint32_t i = 0; i < len; i++) {
    	while(!REG_READ_BIT(pSPI->SR, SPI_SR_TXE_Pos));
    	REG_WRITE(pSPI->DR, buffer[i]);
    }

    while(!REG_READ_BIT(pSPI->SR, SPI_SR_TXE_Pos)); // wait for cmd to be sent
    while(REG_READ_BIT(pSPI->SR, SPI_SR_BSY_Pos));  // wait for the SPI session to finish
    data = REG_READ(pSPI->DR) & 0xFF;

    LCD_CSX_HIGH();
}
#else
void lcd_write_data(uint8_t *buffer, uint32_t len)
{
    SPI_TypeDef *pSPI = SPI;
    uint8_t data;

    for (uint32_t i = 0; i < len; i++) {
    	LCD_CSX_LOW();
    	while(!REG_READ_BIT(pSPI->SR, SPI_SR_TXE_Pos));
    	REG_WRITE(pSPI->DR, buffer[i]);
        while(!REG_READ_BIT(pSPI->SR, SPI_SR_TXE_Pos)); // wait for cmd to be sent
        while(REG_READ_BIT(pSPI->SR, SPI_SR_BSY_Pos));  // wait for the SPI session to finish
        data = REG_READ(pSPI->DR) & 0xFF;
        LCD_CSX_HIGH();
    }
}
#endif
void lcd_buffer_init(bsp_lcd_t *lcd)
{
    lcd->draw_buffer1 = bsp_db;
    lcd->draw_buffer2 = bsp_wb;
    lcd->buff_to_draw = NULL;
    lcd->buff_to_flush = NULL;
}

void lcd_set_display_area(lcd_area_t *area)
{
    uint8_t params[4];

    // Column address set (2Ah)
    params[0] = (area->x1 >> 8U) & 0xFF;
    params[1] = (area->x1 >> 0U) & 0xFF;
    params[2] = (area->x2 >> 8U) & 0xFF;
    params[3] = (area->x2 >> 0U) & 0xFF;
    lcd_write_cmd(ILI9341_CASET);
    lcd_write_data(params, 4);

    params[0] = (area->y1 >> 8U) & 0xFF;
    params[1] = (area->y1 >> 0U) & 0xFF;
    params[2] = (area->y2 >> 8U) & 0xFF;
    params[3] = (area->y2 >> 0U) & 0xFF;
    lcd_write_cmd(ILI9341_RASET);
    lcd_write_data(params, 4);
}

void lcd_set_orientation(uint8_t orientation)
{
	uint8_t param;

	if (orientation == LANDSCAPE) {
		param = MADCTL_MV | MADCTL_MY | MADCTL_BGR; /*Memory Access Control <Landscape setting>*/
	}else if(orientation == PORTRAIT){
		param = MADCTL_MY| MADCTL_MX| MADCTL_BGR;  /* Memory Access Control <portrait setting> */
	}

	lcd_write_cmd(ILI9341_MAC);    // Memory Access Control command
	lcd_write_data(&param, 1);
}

void bsp_lcd_send_cmd_mem_write(void)
{
	lcd_write_cmd(ILI9341_GRAM);
}

void lcd_pin_init(void)
{
	RCC_TypeDef *pRCC = RCC;
	GPIO_TypeDef *pGPIOA = GPIOA;
	GPIO_TypeDef *pGPIOB = GPIOB;

	// Enable RCC clocks for A and B GPIO
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos);

	/*
	 * For every pin use, configure
	 * Enable gen purpose output mode (MODE)
	 * output type (push-pull)
	 * speed high, no pullup/down
	 * Conf pins of SPI pins to AF
	 */

	//PA5 - SCK
	REG_SET_VAL(pGPIOA->MODER, 0x2, 0x3, GPIO_MODER_MODE5_Pos);
	REG_CLR_BIT(pGPIOA->OTYPER, GPIO_OTYPER_OT5_Pos);
	REG_SET_VAL(pGPIOA->OSPEEDR, 0x2, 0x3, GPIO_OSPEEDR_OSPEED5_Pos);
	REG_SET_VAL(pGPIOA->AFR[0], 0x5, 0xF, GPIO_AFRL_AFSEL5_Pos);

	//PA6 - MISO
	REG_SET_VAL(pGPIOA->MODER, 0x2, 0x3, GPIO_MODER_MODE6_Pos);
	REG_CLR_BIT(pGPIOA->OTYPER, GPIO_OTYPER_OT6_Pos);
	REG_SET_VAL(pGPIOA->OSPEEDR, 0x2, 0x3, GPIO_OSPEEDR_OSPEED6_Pos);
	REG_SET_VAL(pGPIOA->AFR[0], 0x5, 0xF, GPIO_AFRL_AFSEL6_Pos);

	//PA7 - MOSI
	REG_SET_VAL(pGPIOA->MODER, 0x2, 0x3, GPIO_MODER_MODE7_Pos);
	REG_CLR_BIT(pGPIOA->OTYPER, GPIO_OTYPER_OT7_Pos);
	REG_SET_VAL(pGPIOA->OSPEEDR, 0x2, 0x3, GPIO_OSPEEDR_OSPEED7_Pos);
	REG_SET_VAL(pGPIOA->AFR[0], 0x5, 0xF, GPIO_AFRL_AFSEL7_Pos);

	//PA1 - RST
	REG_SET_VAL(pGPIOA->MODER, 0x1, 0x3, GPIO_MODER_MODE1_Pos);
	REG_CLR_BIT(pGPIOA->OTYPER, GPIO_OTYPER_OT1_Pos);
	REG_SET_VAL(pGPIOA->OSPEEDR, 0x2, 0x3, GPIO_OSPEEDR_OSPEED1_Pos);

	//PB3 - DCX
	REG_SET_VAL(pGPIOB->MODER, 0x1, 0x3, GPIO_MODER_MODE3_Pos);
	REG_CLR_BIT(pGPIOB->OTYPER, GPIO_OTYPER_OT3_Pos);
	REG_SET_VAL(pGPIOB->OSPEEDR, 0x2, 0x3, GPIO_OSPEEDR_OSPEED3_Pos);

	//PB5 - NCS/CSX
	REG_SET_VAL(pGPIOB->MODER, 0x1, 0x3, GPIO_MODER_MODE5_Pos);
	REG_CLR_BIT(pGPIOB->OTYPER, GPIO_OTYPER_OT5_Pos);
	REG_SET_VAL(pGPIOB->OSPEEDR, 0x2, 0x3, GPIO_OSPEEDR_OSPEED5_Pos);

	//CSX = HIGH
	REG_SET_BIT(pGPIOB->ODR, LCD_CSX_PIN);
	//DCX = HIGH
	REG_SET_BIT(pGPIOB->ODR, LCD_DCX_PIN);
	//RESX=HIGH
	REG_SET_BIT(pGPIOA->ODR, LCD_RESX_PIN);
}

void lcd_spi_init(void)
{
	SPI_TypeDef *pSPI = SPI;
	RCC_TypeDef *pRCC = RCC;

	REG_SET_BIT(pRCC->APB2ENR, RCC_APB2ENR_SPI1EN_Pos);

	REG_SET_BIT(pSPI->CR1, SPI_CR1_MSTR_Pos);
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_BIDIMODE_Pos);
	REG_SET_BIT(pSPI->CR1, SPI_CR1_BIDIOE_Pos);
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_DFF_Pos);
	REG_SET_BIT(pSPI->CR1, SPI_CR1_SSM_Pos);
	REG_SET_BIT(pSPI->CR1, SPI_CR1_SSI_Pos);
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_LSBFIRST_Pos);	// Send MSB first
	REG_SET_VAL(pSPI->CR1, 0x1, 0x7, SPI_CR1_BR_Pos);
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_CPOL_Pos);
	REG_CLR_BIT(pSPI->CR1, SPI_CR1_CPHA_Pos);
	REG_CLR_BIT(pSPI->CR1, SPI_CR2_FRF_Pos);		// motorola format
}

void lcd_spi_enable(void)
{
	__enable_spi();
}

void lcd_reset(void)
{
	// pull RESX pin low for some time, then pull high
	LCD_RESX_LOW();
	for (uint32_t i = 0; i < (0xFFFFU * 10U); i++);

	LCD_RESX_HIGH();
	for (uint32_t i = 0; i < (0xFFFFU * 10U); i++);
}

void lcd_config(void)
{
	uint8_t params[15];

	lcd_write_cmd(ILI9341_SWRESET);

	lcd_write_cmd(ILI9341_POWERB);
	params[0] = 0x00;
	params[1] = 0xD9;
	params[2] = 0x30;
	lcd_write_data(params, 3);

	lcd_write_cmd(ILI9341_POWER_SEQ);
	params[0]= 0x64;
	params[1]= 0x03;
	params[2]= 0x12;
	params[3]= 0x81;
	lcd_write_data(params, 4);

	lcd_write_cmd(ILI9341_DTCA);
	params[0]= 0x85;
	params[1]= 0x10;
	params[2]= 0x7A;
	lcd_write_data(params, 3);

	lcd_write_cmd(ILI9341_POWERA);
	params[0]= 0x39;
	params[1]= 0x2C;
	params[2]= 0x00;
	params[3]= 0x34;
	params[4]= 0x02;
	lcd_write_data(params, 5);

	lcd_write_cmd(ILI9341_PRC);
	params[0]= 0x20;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_DTCB);
	params[0]= 0x00;
	params[1]= 0x00;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_POWER1);
	params[0]= 0x1B;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_POWER2);
	params[0]= 0x12;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_VCOM1);
	params[0]= 0x08;
	params[1]= 0x26;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_VCOM2);
	params[0]= 0xB7;
	lcd_write_data(params, 1);


	lcd_write_cmd(ILI9341_PIXEL_FORMAT);
	params[0]= 0x55; //select RGB565
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_FRMCTR1);
	params[0]= 0x00;
	params[1]= 0x1B;//frame rate = 70
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_DFC);    // Display Function Control
	params[0]= 0x0A;
	params[1]= 0xA2;
	lcd_write_data(params, 2);

	lcd_write_cmd(ILI9341_3GAMMA_EN);    // 3Gamma Function Disable
	params[0]= 0x02; //LCD_WR_DATA(0x00);
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_GAMMA);
	params[0]= 0x01;
	lcd_write_data(params, 1);

	lcd_write_cmd(ILI9341_PGAMMA);    //Set Gamma
	params[0]= 0x0F;
	params[1]= 0x1D;
	params[2]= 0x1A;
	params[3]= 0x0A;
	params[4]= 0x0D;
	params[5]= 0x07;
	params[6]= 0x49;
	params[7]= 0X66;
	params[8]= 0x3B;
	params[9]= 0x07;
	params[10]= 0x11;
	params[11]= 0x01;
	params[12]= 0x09;
	params[13]= 0x05;
	params[14]= 0x04;
	lcd_write_data(params, 15);

	lcd_write_cmd(ILI9341_NGAMMA);
	params[0]= 0x00;
	params[1]= 0x18;
	params[2]= 0x1D;
	params[3]= 0x02;
	params[4]= 0x0F;
	params[5]= 0x04;
	params[6]= 0x36;
	params[7]= 0x13;
	params[8]= 0x4C;
	params[9]= 0x07;
	params[10]= 0x13;
	params[11]= 0x0F;
	params[12]= 0x2E;
	params[13]= 0x2F;
	params[14]= 0x05;
	lcd_write_data(params, 15);

	lcd_write_cmd(ILI9341_SLEEP_OUT); //Exit Sleep
	delay_50ms();
	delay_50ms();
	lcd_write_cmd(ILI9341_DISPLAY_ON); //display on
}

void bsp_lcd_write(uint8_t *buffer, uint32_t nbytes)
{
	uint16_t *buff_ptr;

	__disable_spi();
	__spi_set_dff_16bit();
	__enable_spi();

	LCD_CSX_LOW();
	buff_ptr = (uint16_t*)buffer;
	while (nbytes) {
		while(!REG_READ_BIT(SPI->SR, SPI_SR_TXE_Pos));
		REG_WRITE(SPI->DR, *buff_ptr);
		++buff_ptr;
		nbytes -= 2;
	}

	while(REG_READ_BIT(SPI->SR,SPI_SR_BSY_Pos));
	__disable_spi();
	LCD_CSX_HIGH();
	__spi_set_dff_8bit();
	__enable_spi();
}

static uint8_t is_lcd_write_allowed(bsp_lcd_t *hlcd)
{
	// buff_to_draw is critical section, so interrupts must be disabled first
	// when reading and enabled again after
	__disable_irq();
	if(!hlcd->buff_to_flush)
		return 1;

	__enable_irq();

	// DMA still ongoing, buff_to_draw not yet available
	return 0;
}

 void lcd_flush(bsp_lcd_t *hlcd)
{
	lcd_set_display_area(&hlcd->area);
	bsp_lcd_send_cmd_mem_write();
#if (USE_DMA == 0)
	bsp_lcd_write(hlcd->buff_to_flush,hlcd->write_length);
	hlcd->buff_to_flush = NULL;
#else
	lcd_write_dma((uint32_t)hlcd->buff_to_flush,hlcd->write_length);
#endif
}

/* get pixels number given number of bytes.  assumes RGB565, so divide by 2 */
uint32_t bytes_to_pixels(uint32_t nbytes, uint32_t pixel_format)
{
	return (nbytes / 2);
}

uint32_t pixels_to_bytes(uint32_t pixels, uint32_t pixel_format)
{
	return (pixels * 2);
}

/* RGB888 - red [23:16] grn [15:8] blue[7:0]
 * Get 5MSB for blue, 6MSB green, 5MSB red
 */
uint16_t convert_rgb888_to_rgb565(uint32_t rgb888)
{
    uint16_t r,g,b;
	r = (rgb888 >> 19) & 0x1FU;
	g = (rgb888 >> 10) & 0x3FU;
	b = (rgb888 >> 3)  & 0x1FU;
	return (uint16_t)((r << 11) | (g << 5) | b);
}

uint8_t *get_buff(bsp_lcd_t *hlcd)
{
    uint32_t buf1 = (uint32_t)hlcd->draw_buffer1;
    uint32_t buf2 = (uint32_t)hlcd->draw_buffer2;

    __disable_irq();

    if (hlcd->buff_to_draw == NULL && hlcd->buff_to_flush == NULL) {
    	return hlcd->draw_buffer1;
    } else if ((uint32_t)hlcd->buff_to_flush == buf1 && hlcd->buff_to_draw == NULL) {
    	return hlcd->draw_buffer2;
    } else if ((uint32_t)hlcd->buff_to_flush == buf2 && hlcd->buff_to_draw == NULL) {
    	return hlcd->draw_buffer1;
    }

    __enable_irq();

    return NULL;
}

uint32_t copy_to_draw_buffer(bsp_lcd_t *hlcd, uint32_t nbytes, uint32_t rgb888)
{
	uint16_t *fb_ptr = NULL;
	uint32_t npixels;

	hlcd->buff_to_draw = get_buff(hlcd);
	fb_ptr = (uint16_t *)hlcd->buff_to_draw;
	nbytes = ((nbytes > DB_SIZE) ? DB_SIZE : nbytes);

	npixels = bytes_to_pixels(nbytes, hlcd->pixel_format);
	if (hlcd->buff_to_draw != NULL) {
		for (uint32_t i = 0; i < npixels; i++) {
			*fb_ptr = convert_rgb888_to_rgb565(rgb888);
			fb_ptr++;
		}

		hlcd->write_length = pixels_to_bytes(npixels, hlcd->pixel_format);

		while(!is_lcd_write_allowed(hlcd));
		hlcd->buff_to_flush = hlcd->buff_to_draw;
		hlcd->buff_to_draw = NULL;
		lcd_flush(hlcd);
		return pixels_to_bytes(npixels, hlcd->pixel_format);
	}

	return 0;
}

void make_area(lcd_area_t *area, uint32_t x_start, uint32_t x_width,
		       uint32_t y_start, uint32_t y_height)
{
	uint16_t lcd_total_width, lcd_total_height;

	lcd_total_width = BSP_LCD_ACTIVE_WIDTH - 1;
	lcd_total_height = BSP_LCD_ACTIVE_HEIGHT - 1;

	area->x1 = x_start;
	area->x2 = x_start + x_width - 1;
	area->y1 = y_start;
	area->y2 = y_start + y_height - 1;

	// making sure x2 and y2 are within limits
	area->x2 = (area->x2 > lcd_total_width) ? lcd_total_width : area->x2;
	area->y2 = (area->y2 > lcd_total_height) ? lcd_total_height : area->y2;
}

/* compute total bytes of pixels to send, for RGB565 */
uint32_t get_total_bytes(bsp_lcd_t *hlcd, uint32_t w, uint32_t h)
{
	uint8_t bytes_per_pixel = 2;

	if (hlcd->pixel_format == BSP_LCD_PIXEL_FMT_RGB565) {
		bytes_per_pixel = 2;
	}

	return (w * h * bytes_per_pixel);
}

void bsp_lcd_fill_rect(uint32_t rgb888, uint32_t x_start, uint32_t x_width,
		               uint32_t y_start, uint32_t y_height)
{
	uint32_t total_bytes_to_write = 0;
	uint32_t bytes_sent_so_far = 0;
	uint32_t remaining_bytes = 0;
	uint32_t npix;
	uint32_t pixels_sent = 0;
	uint32_t x1, y1;
	uint32_t pixel_per_line = x_width;

	if ((x_start + x_width) > BSP_LCD_ACTIVE_WIDTH)
		return;

	if ((y_start + y_height) > BSP_LCD_ACTIVE_HEIGHT)
		return;

	total_bytes_to_write = get_total_bytes(hlcd, x_width, y_height);
	remaining_bytes = total_bytes_to_write;
	while (remaining_bytes) {
		x1 = x_start + (pixels_sent % pixel_per_line);
		y1 = y_start + (pixels_sent / pixel_per_line);

		make_area(&hlcd->area, x1, x_width, y1, y_height);

		if (x1 != x_start) {
			npix = x_start + x_width - x1;
		} else {
			npix = bytes_to_pixels(remaining_bytes, hlcd->pixel_format);
		}

		bytes_sent_so_far += copy_to_draw_buffer(hlcd, pixels_to_bytes(npix, hlcd->pixel_format), rgb888);
		pixels_sent = bytes_to_pixels(bytes_sent_so_far, hlcd->pixel_format);
		remaining_bytes = total_bytes_to_write - bytes_sent_so_far;

	}
}

void bsp_lcd_set_background_color(uint32_t rgb888)
{
	// Colors are still by default RGB888-based
	bsp_lcd_fill_rect(rgb888, 0, BSP_LCD_ACTIVE_WIDTH, 0, BSP_LCD_ACTIVE_HEIGHT);
}

void DMA2_Stream3_IRQHandler(void)
{
	uint32_t tmp;
	DMA_TypeDef *pDMA = DMA2;

	tmp = pDMA->LISR;
	if (REG_READ_BIT(tmp, DMA_LISR_TCIF3_Pos)) {
		REG_SET_BIT(pDMA->LIFCR, DMA_LIFCR_CTCIF3_Pos);
		dma_cmplt_callback_spi_write(&lcd_handle);
	} else if (REG_READ_BIT(tmp, DMA_LISR_TEIF3_Pos)) {
		REG_SET_BIT(pDMA->LIFCR, DMA_LIFCR_CTEIF3_Pos);
		dma_lcd_write_error(&lcd_handle);
	} else if (REG_READ_BIT(tmp, DMA_LISR_FEIF3_Pos)) {
		REG_SET_BIT(pDMA->LIFCR, DMA_LIFCR_CFEIF3_Pos);
		dma_lcd_write_error(&lcd_handle);
	}
}

void bsp_lcd_init(void)
{
	lcd_pin_init();
	lcd_spi_init();
	lcd_spi_enable();
	lcd_handle.orientation = BSP_LCD_ORIENTATION;
    lcd_handle.pixel_format = BSP_LCD_PIXEL_FMT;
	lcd_reset();
	lcd_config();

	hlcd->area.x1 = 0;
	hlcd->area.x2 = BSP_LCD_ACTIVE_WIDTH - 1;
	hlcd->area.y1 = 0;
	hlcd->area.y2 = BSP_LCD_ACTIVE_HEIGHT - 1;

	lcd_set_display_area(&hlcd->area);
	lcd_set_orientation(hlcd->orientation);
	lcd_buffer_init(hlcd);
	lcd_dma_init(hlcd);
}

