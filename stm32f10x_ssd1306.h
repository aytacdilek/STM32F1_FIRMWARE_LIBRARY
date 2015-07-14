/*******************************************************************************
* File  		:	stm32f10x_htu21.h
* Description	: 	STM32F10x library for HTU21 Digital Humidity and Temperature Sensor
* Author		: 	Aytac Dilek
* Note			: 	GNU General Public License, version 3 (GPL-3.0)
*******************************************************************************/

#ifndef __STM32F10X_SSD1306_H
#define __STM32F10X_SSD1306_H


/* Includes */
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"

#include "open103z_config.h"
#include "open103z_systick.h"


/* Defines */
#define SSD1306_I2C					I2C2
#define SSD1306_I2C_CLOCK			RCC_APB1Periph_I2C2
#define SSD1306_GPIO_CLOCK			RCC_APB2Periph_GPIOB
#define SSD1306_I2C_PORT			GPIOB
#define SSD1306_I2C_SDA_PIN			GPIO_Pin_11
#define SSD1306_I2C_SCL_PIN			GPIO_Pin_10
#define SSD1306_I2C_CLOCK_SPEED		100000

#define SSD1306_I2C_ADDRESS				0x78	/* Shifted 8-bit I2C address for the sensor */

/* I2C Commands */
#define SSD1306_COMMAND_SEND			0x80	/* Send command */
#define SSD1306_DATA_SEND				0x40	/* Send data */

#define SSD1306_COMMAND_SET_CONTRAST			0x81
#define SSD1306_COMMAND_DISPLAYALLON_RESUME		0xA4
#define SSD1306_COMMAND_DISPLAYALLON			0xA5
#define SSD1306_COMMAND_NORMALDISPLAY			0xA6
#define SSD1306_COMMAND_INVERTDISPLAY			0xA7
#define SSD1306_COMMAND_DISPLAY_OFF				0xAE	/* Turns display off. */
#define SSD1306_COMMAND_DISPLAY_ON				0xAF	/* Turns display on. */
#define SSD1306_COMMAND_SETDISPLAYOFFSET 		0xD3
#define SSD1306_COMMAND_SETCOMPINS 				0xDA
#define SSD1306_COMMAND_SETVCOMDETECT 			0xDB
#define SSD1306_COMMAND_SETDISPLAYCLOCKDIV 		0xD5
#define SSD1306_COMMAND_SETPRECHARGE 			0xD9
#define SSD1306_COMMAND_SETMULTIPLEX 			0xA8
#define SSD1306_COMMAND_SETLOWCOLUMN 			0x00
#define SSD1306_COMMAND_SETHIGHCOLUMN 			0x10
#define SSD1306_COMMAND_SETSTARTLINE 			0x40
#define SSD1306_COMMAND_MEMORYMODE 				0x20
#define SSD1306_COMMAND_COLUMNADDR 				0x21
#define SSD1306_COMMAND_PAGEADDR 				0x22
#define SSD1306_COMMAND_COMSCANINC 				0xC0
#define SSD1306_COMMAND_COMSCANDEC 				0xC8
#define SSD1306_COMMAND_SEGREMAP 				0xA0
#define SSD1306_COMMAND_CHARGEPUMP 				0x8D
#define SSD1306_COMMAND_EXTERNALVCC 			0x1
#define SSD1306_COMMAND_SWITCHCAPVCC 			0x2
// Scrolling #defines
#define SSD1306_COMMAND_ACTIVATE_SCROLL 						0x2F
#define SSD1306_COMMAND_DEACTIVATE_SCROLL 						0x2E
#define SSD1306_COMMAND_SET_VERTICAL_SCROLL_AREA				0xA3
#define SSD1306_COMMAND_RIGHT_HORIZONTAL_SCROLL 				0x26
#define SSD1306_COMMAND_LEFT_HORIZONTAL_SCROLL 					0x27
#define SSD1306_COMMAND_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL	0x29
#define SSD1306_COMMAND_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 	0x2A

/* Pixel size definitions */
#define SSD1306_128_64
//#define SSD1306_128_32

#if defined SSD1306_128_64
	#define SSD1306_LCD_WIDTH		128
	#define SSD1306_LCD_HEIGHT		64
#endif
#if defined SSD1306_128_32
	#define SSD1306_LCD_WIDTH		128
	#define SSD1306_LCD_HEIGHT		32
#endif

/* Enumarations */
typedef enum{
	BLACK = 0,
	WHITE = 1
}Color_TypeDef;


/* Global Variables */


/* Global Functions */
void ssd1306_init(void);
void ssd1306_sendCommand(uint8_t command);
void ssd1306_displayOff(void);
void ssd1306_displayOn(void);
void ssd1306_sendChar(uint8_t data);
void ssd1306_setXY(uint8_t row, uint8_t column);
void ssd1306_clearDisplay(void);
void ssd1306_resetDisplay(void);


#endif // __STM32F10X_SSD1306_H
