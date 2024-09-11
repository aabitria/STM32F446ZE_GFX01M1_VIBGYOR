# Basic LCD-TFT Display Control

This project is just for barebones control of LCD-TFT graphics, ported to NUCLEO-F446ZE.  No HAL code is used for low level code, just pure register read/write accesses.  It does the following:
- initialize system clock
- initialize SPI controller
- configure the LCD driver
- Display the ROYGBIV colors.

Hardware Used:
- NUCLEO-F446ZE devkit
- X-NUCLEO-GFX01M1 devkit connected to the nucleo board, with ILI9341 controlling a 240 x 320 LCD

The LCD in GFX01M1 is connected to the STM32F446 MCU via SPI lines for data and control and via some GPIOs for control.  No 'framebuffer' is allocated since the overall buffer will be above the total SRAM.  Instead, partial buffering is used.  LCD data is being DMA'ed over the SPI bus to the LCD for faster response.
