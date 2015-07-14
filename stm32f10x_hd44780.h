// My 16x2 HD44780 LCD Driver, based entirely on the flex_lcd.c
// driver by "PCM Programmer" of the CCS Info Forum
// http://www.ccsinfo.com/forum/viewtopic.php?t=24661
//
// It has just been slightly modified to suit my purposes, and
// documented for my own reference.  Hopefully it will prove
// useful to others.
//
// This driver ONLY supports 6-wire communications, so the LCD
// R/W (pin 5) must be tied to GND, and only 4 data lines are
// used (D4-D7) along with RS and Enable.
//
//  1              16
//  ________________________________________
// |oooooooooooooooo                        |
// |   __________________________________   |
// |  | [][][][][][][][][][][][][][][][] |  |
// |  | [][][][][][][][][][][][][][][][] |  |
// |  |__________________________________|  |
// |________________________________________|
//  Front View of a typical 16-wire 16x2 LCD
//
// The wiring for the LCD is as follows:
// 1 : GND
// 2 : 5V
// 3 : Contrast (0-5V)*
// 4 : RS (Register Select)
// 5 : R/W (Read Write)       - GROUND THIS PIN
// 6 : Enable or Strobe
// 7 : Data Bit 0             - NOT USED
// 8 : Data Bit 1             - NOT USED
// 9 : Data Bit 2             - NOT USED
// 10: Data Bit 3             - NOT USED
// 11: Data Bit 4
// 12: Data Bit 5
// 13: Data Bit 6
// 14: Data Bit 7
// 15: LCD Backlight +5V**
// 16: LCD Backlight GND
//
// *A 10k potentiometer with the two outside legs across 5V
// and GND, with the center (wiper) connected to this pin
// works perfectly here.
//
// **Some backlighting on some LCDs does not incorporate a
// current limiting resistor - check to see if yours does - if
// not, add an approx 180R resistor between 5V and pin 15.
// Alternatively, some LCDs hard-wire the backlighting on the
// board and provide a jumper you can remove if you don't want
// it... check the datasheet for your display.
//
// LCD Pin Assignments:
// The following are configured to suit the PIC16F628A so the
// 4 data pins (B4-B7) align with the 4 I/O pins used.  If
// you want to use other I/O pins then define the ones you
// want in your code, before including this file so that
// they override these ones.
#ifndef __STM32F0XX_HD44780_H
#define __STM32F0XX_HD44780_H


/* Includes */
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "open103z_systick.h"



/*	Pin initialization of control and data signals */
#define LCD_CTRL_RS_PIN            		GPIO_Pin_0     			/* PA.0 */
#define LCD_CTRL_RS_GPIO_PORT      	 	GPIOA					/* GPIOA */
#define LCD_CTRL_RS_GPIO_CLK        	RCC_APB2Periph_GPIOA	/* GPIOA Clock */
#define LCD_CTRL_RS_SOURCE          	GPIO_PinSource0			/* Pin Mask 0 */

#define LCD_CTRL_RW_PIN            		GPIO_Pin_1     			/* PA.1 */
#define LCD_CTRL_RW_GPIO_PORT      	 	GPIOA               	/* GPIOB */
#define LCD_CTRL_RW_GPIO_CLK        	RCC_APB2Periph_GPIOA	/* GPIOA Clock */
#define LCD_CTRL_RW_SOURCE          	GPIO_PinSource1			/* Pin Mask 1 */

#define LCD_CTRL_EN_PIN           		GPIO_Pin_2     			/* PA.2 */
#define LCD_CTRL_EN_GPIO_PORT      		GPIOA               	/* GPIOB */
#define LCD_CTRL_EN_GPIO_CLK      		RCC_APB2Periph_GPIOA	/* GPIOA Clock */
#define LCD_CTRL_EN_SOURCE         		GPIO_PinSource2			/* Pin Mask 2 */

#define LCD_DATA_BIT_4_PIN				GPIO_Pin_4       		/* PA.4 */
#define LCD_DATA_BIT_4_GPIO_PORT       	GPIOA                	/* GPIOC */
#define LCD_DATA_BIT_4_GPIO_CLK        	RCC_APB2Periph_GPIOA	/* GPIOA Clock */
#define LCD_DATA_BIT_4_SOURCE          	GPIO_PinSource4			/* Pin Mask 4 */

#define LCD_DATA_BIT_5_PIN         		GPIO_Pin_5           	/* PA.5 */
#define LCD_DATA_BIT_5_GPIO_PORT       	GPIOA                 	/* GPIOC */
#define LCD_DATA_BIT_5_GPIO_CLK        	RCC_APB2Periph_GPIOA	/* GPIOA Clock */
#define LCD_DATA_BIT_5_SOURCE          	GPIO_PinSource5			/* Pin Mask 5 */

#define LCD_DATA_BIT_6_PIN         		GPIO_Pin_6        		/* PA.6 */
#define LCD_DATA_BIT_6_GPIO_PORT       	GPIOA               	/* GPIOC */
#define LCD_DATA_BIT_6_GPIO_CLK        	RCC_APB2Periph_GPIOA	/* GPIOA Clock */
#define LCD_DATA_BIT_6_SOURCE          	GPIO_PinSource6			/* Pin Mask 6 */

#define LCD_DATA_BIT_7_PIN            	GPIO_Pin_7     			/* PA.7 */
#define LCD_DATA_BIT_7_GPIO_PORT       	GPIOA               	/* GPIOC */
#define LCD_DATA_BIT_7_GPIO_CLK        	RCC_APB2Periph_GPIOA	/* GPIOA Clock */
#define LCD_DATA_BIT_7_SOURCE      		GPIO_PinSource7			/* Pin Mask 7 */



/* Constant Definitions */
#define LCD_LINE_LENGTH					(uint8_t)16
#define LCD_LINE_TWO_ADDR				(uint8_t)64



/*
 * @brief:	LCD Commands
 *
 * Instruction					RS		R/W		DB7		DB6		DB5		DB4		DB3		DB2		DB1		DB0
 * Clear Display:				0	|	0	|	0	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|
 * Return Home:					0	|	0	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	*	|
 * Entry Mode Set:				0	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	I/D	|	S	|
 * Display On/Off:				0	|	0	|	0	|	0	|	0	|	0	|	1	|	D	|	C	|	B	|
 * Shift Cursor:				0	|	0	|	0	|	0	|	0	|	1	|	S/C	|	R/L	|	*	|	*	|
 * Set Function:				0	|	0	|	0	|	0	|	1	|	DL	|	N	|	F	|	*	|	*	|
 * Set CG RAM address:			0	|	0	|	0	|	1	|					Acg
 * Set DD RAM address:			0	|	0	|	1	|							Add
 * Read busy flag & address:	0	|	1	|	BF	|							AC
 * Write data to CG/DD RAM:		1	|	0	|								Write Data
 * Read data from CG/DD RAM:	1	|	1	|								Read Data
 */
#define LCD_CLEAR_DISPLAY				0x01		// Clears all display and returns the cursor to home position (Address 0)

#define LCD_RETURN_HOME					0x02		// Returns the cursor to the home position (Address 0).
													// Also returns the display being shifted to the original position. DDRAM contents remain unchanged.

#define LCD_ENTRY_MODE_SET				0x04		// Sets the cursor move direction and specifies or not to shift the display.
													// These operations are performed during data write and read
#define LCD_CURSOR_INCREMENT			0x02		// I/D=1: Increment (+1), I/D=0: Decrement (-1)
													// S=1:	Accompanies display shift

#define LCD_SET_DISPLAY					0x08		// Sets ON/OFF of all display (D), cursor ON/OFF (C), and blink of cursor position character (B).
#define LCD_DISPLAY_ON					0x04		// D=1: Display ON
#define LCD_CURSOR_ON					0x02		// C=1: Cursor ON
#define LCD_BLINK_ON					0x01		// B=1: Blink ON

#define LCD_SHIFT_CURSOR				0x10		// Moves the cursor and shiftes the display without changing DD RAM contents.
													// S/C=1: Display shift S/C=0: Cursor move
													// R/L=1: Shift to right
													// R/L=0: Shift to left
#define LCD_SET_FUNCTION				0x20		// Sets interface data length (DL)number of display lines (L) and character font (F).
#define LCD_SET_DATA_LENGTH_4			0x00		// DL =1: 8 bits DL =0: 4 bits
#define LCD_SET_DATA_LENGTH_8			0x10
#define LCD_SET_NUMBER_OF_LINES_2		0x80		// N =1: 2 lines N =0: 1 lines
#define LCD_SET_NUMBER_OF_LINES_1		0x00
#define LCD_SET_FONT_SIZE_5X10			0x40		// F =1: 5x10 dots F =0: 5x7 dots
#define LCD_SET_FONT_SIZE_5X7			0x00

#define LCD_SET_CGRAM_ADDRESS			0x40		// Sets the CG RAM address.CG RAM data is sent and received. After this setting.
#define LCD_SET_DDRAM_ADDRESS			0x80		// Sets the DD address. DD RAM data is sent and received After this setting.



/* Macro Definitions */
#define LCD_EN_LOW()					GPIO_ResetBits(LCD_CTRL_EN_GPIO_PORT, LCD_CTRL_EN_PIN)		/* Enable Selection */
#define LCD_EN_HIGH()					GPIO_SetBits(LCD_CTRL_EN_GPIO_PORT, LCD_CTRL_EN_PIN)		/* Disable Selection */

#define LCD_RS_LOW()					GPIO_ResetBits(LCD_CTRL_RS_GPIO_PORT, LCD_CTRL_RS_PIN)		/* Instruction Register Selection */
#define LCD_RS_HIGH()					GPIO_SetBits(LCD_CTRL_RS_GPIO_PORT, LCD_CTRL_RS_PIN)		/* Data Register Selection */

#define LCD_RS(x)						GPIO_WriteBit(LCD_CTRL_RS_GPIO_PORT, LCD_CTRL_RS_PIN, x)	/* Register Selection */
#define LCD_EN(x)						GPIO_WriteBit(LCD_CTRL_EN_GPIO_PORT, LCD_CTRL_EN_PIN, x)	/* Enable Selection */

#define GET_BIT(a, b)					((a >> b) & 1)



/*  Global Function Definitions  */
void LCD_Init(void);			/* Initialize LCD and peripheral */
void LCD_Clear(void);
void LCD_Goto(uint8_t cy, uint8_t cx);	/* cx is column number, cy is row number */
void LCD_WriteCustomCharacter(uint8_t num, const uint8_t *c);
void LCD_Putc(char c);
void LCD_Puts(const char *s);
void LCD_PutSignedInt(int32_t value);
void LCD_PutUnsignedInt(uint32_t value);

/* Internal Function Definitions */
void LCD_LowLevelInit();
void LCD_SendNibble(uint8_t n);
void LCD_SendByte(uint8_t address, uint8_t n);
void LCD_SetDdramAddress(uint8_t address);
void LCD_SetCgramAddress(uint8_t address);
//void LCD_EntryModeCommand(LCD_EntryModeCmdTypeDef *LCD_EntryModeCmdStruct);
//void LCD_DisplayOnOffCommand(LCD_DisplayOnOffCmdTypedef *LCD_DisplayOnOffStruct);
//void LCD_CursorDisplayShiftCommand(LCD_CursorDisplayShiftCmdTypeDef *LCD_CursorDisplayShiftStruct);

#endif	//__STM32F0XX_HD44780_H
