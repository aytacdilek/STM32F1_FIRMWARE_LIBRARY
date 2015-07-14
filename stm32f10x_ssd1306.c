/*
 * File			:	stm32f10x_ssd1306.c
 * Description	:	STM32F10x library for Monochrome 0.96" 128x64 OLED graphic display
 * Datum		:	2015.03.07
 * Version		:	1.0
 * Author		:	Aytac Dilek
 * email		:	aytacdilek@gmail.com
 * Web			:
 * Platform		:	OPEN103Z-B
 * CPU			:	STM32F103ZET6
 * IDE			:	CooCox CoIDE 1.7.7
 * GCC			:	4.8 2014q2
 * Module		:	0.96" 128x64 OLED LCD with SSD1306 controller
 * Function		:	Displayer
 */


/* Includes */
#include "stm32f10x_ssd1306.h"


/*******************************************************************************
* Function Name  : htu21_init
* Description    : Initialize display sensor
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void ssd1306_init(void){
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(SSD1306_I2C_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(SSD1306_GPIO_CLOCK | RCC_APB2Periph_AFIO , ENABLE);

	/* Configure I2C1 pins: PB6->SCL and PB7->SDA */
	GPIO_InitStructure.GPIO_Pin =  SSD1306_I2C_SCL_PIN | SSD1306_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(SSD1306_I2C_PORT, &GPIO_InitStructure);

	I2C_DeInit(SSD1306_I2C);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = SSD1306_I2C_CLOCK_SPEED;
	I2C_Init(SSD1306_I2C, &I2C_InitStructure);

	I2C_Cmd(SSD1306_I2C, ENABLE);

	I2C_AcknowledgeConfig(SSD1306_I2C, ENABLE);

	systick_init();
}


/*******************************************************************************
* Function Name  : ssd1306_sendCommand
* Description    : Used to send commands to the display.
* Input          : unsigned 8-bit data
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void ssd1306_sendCommand(uint8_t command){
	while(I2C_GetFlagStatus(SSD1306_I2C, I2C_FLAG_BUSY));	/* Wait while the bus is busy */
	I2C_AcknowledgeConfig(SSD1306_I2C, ENABLE);		/* Enable Acknowledge */
	I2C_GenerateSTART(SSD1306_I2C, ENABLE);		/* Send "Start" condition */
	while(!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_MODE_SELECT));		/* Test on EV5 and clear it */
	I2C_Send7bitAddress(SSD1306_I2C,  SSD1306_I2C_ADDRESS, I2C_Direction_Transmitter);	/* Send HTU21D address for write */
	while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	/* Test on EV6 and clear it */
	I2C_Cmd(SSD1306_I2C, ENABLE);	/* Clear EV6 by setting again the PE bit */
	I2C_SendData(SSD1306_I2C, SSD1306_COMMAND_SEND);		/* Send command */
	while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));		/* Test on EV8 and clear it */
	I2C_SendData(SSD1306_I2C, command);		/* Send "temperature read" command */
	while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));		/* Test on EV8 and clear it */
	I2C_GenerateSTOP(SSD1306_I2C, ENABLE);		/* Send "Stop" condition */
}


/*******************************************************************************
* Function Name  : ssd1306_displayOff
* Description    : Turns display off.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void ssd1306_displayOff(void){
	ssd1306_sendCommand(SSD1306_COMMAND_DISPLAY_OFF);
}


/*******************************************************************************
* Function Name  : ssd1306_displayOn
* Description    : Turns display on.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void ssd1306_displayOn(void){
	ssd1306_sendCommand(SSD1306_COMMAND_DISPLAY_OFF);
}



/*******************************************************************************
* Function Name  : 	ssd1306_sendChar
* Description    : 	Sends data to display.
* Input          : 	None
* Output         : 	None
* Return         : 	None
* Attention		 : 	Actually this sends a byte, not a char to draw in the display.
* 					Display's chars uses 8 byte font the small ones and
* 					96 bytes for the big number font.
*******************************************************************************/
void ssd1306_sendChar(uint8_t data){
	while(I2C_GetFlagStatus(SSD1306_I2C, I2C_FLAG_BUSY));	/* Wait while the bus is busy */
	I2C_AcknowledgeConfig(SSD1306_I2C, ENABLE);		/* Enable Acknowledge */
	I2C_GenerateSTART(SSD1306_I2C, ENABLE);		/* Send "Start" condition */
	while(!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_MODE_SELECT));		/* Test on EV5 and clear it */
	I2C_Send7bitAddress(SSD1306_I2C,  SSD1306_I2C_ADDRESS, I2C_Direction_Transmitter);	/* Send HTU21D address for write */
	while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	/* Test on EV6 and clear it */
	I2C_Cmd(SSD1306_I2C, ENABLE);	/* Clear EV6 by setting again the PE bit */
	I2C_SendData(SSD1306_I2C, SSD1306_DATA_SEND);		/* Send command */
	while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));		/* Test on EV8 and clear it */
	I2C_SendData(SSD1306_I2C, data);		/* Send "temperature read" command */
	while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));		/* Test on EV8 and clear it */
	I2C_GenerateSTOP(SSD1306_I2C, ENABLE);		/* Send "Stop" condition */
}


/*******************************************************************************
* Function Name  : 	ssd1306_setXY
* Description    : 	Set the cursor position in a 16 COL * 8 ROW map.
* Input          : 	None
* Output         : 	None
* Return         : 	None
* Attention		 : 	None
*******************************************************************************/
void ssd1306_setXY(uint8_t row, uint8_t column){
	ssd1306_sendCommand(0xb0+row);                		//set page address
	ssd1306_sendCommand(0x00+(8*column&0x0f));       	//set low col address
	ssd1306_sendCommand(0x10+((8*column>>4)&0x0f));  	//set high col address
}



/*******************************************************************************
* Function Name  : 	ssd1306_clearDisplay
* Description    : 	Clears the display by sending 0 to all the screen map.
* Input          : 	None
* Output         : 	None
* Return         : 	None
* Attention		 : 	None
*******************************************************************************/
void ssd1306_clearDisplay(void){
	uint8_t i, j;
	for (j = 0; j < 8; ++j) {
		ssd1306_setXY(j, 0);

		for (i = 0; i < 128; ++i) {
			ssd1306_sendChar(0);	// Clear all column
		}
	}
}


/*******************************************************************************
* Function Name  : 	ssd1306_resetDisplay
* Description    : 	Resets display depending on the actual mode.
* Input          : 	None
* Output         : 	None
* Return         : 	None
* Attention		 : 	None
*******************************************************************************/
void ssd1306_resetDisplay(void){
	ssd1306_displayOff();
	ssd1306_clearDisplay();
	ssd1306_displayOn();
}
