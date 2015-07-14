/*
 * File			:	stm32f10x_htu21.c
 * Description	:	STM32F10x library for HTU21 Digital Humidity and Temperature Sensor
 * Datum		:	2015.03.06
 * Version		:	1.0
 * Author		:	Aytac Dilek
 * email		:	aytacdilek@gmail.com
 * Web			:
 * Platform		:	OPEN103Z-B
 * CPU			:	STM32F103ZET6
 * IDE			:	CooCox CoIDE 1.7.7
 * GCC			:	4.8 2014q2
 * Module		:	HTU21D
 * Function		:	Read Temperature and Humidity
 */


/* Includes */
#include "stm32f10x_htu21.h"


/*******************************************************************************
* Function Name  : htu21_init
* Description    : Initialize HTU21 sensor
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void htu21_init(void){
	   I2C_InitTypeDef  I2C_InitStructure;
	   GPIO_InitTypeDef  GPIO_InitStructure;

	   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);

	   /* Configure I2C1 pins: PB6->SCL and PB7->SDA */
	   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	   GPIO_Init(GPIOB, &GPIO_InitStructure);

	   I2C_DeInit(I2C1);
	   I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	   I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	   I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	   I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	   I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	   I2C_InitStructure.I2C_ClockSpeed = 10000;
	   I2C_Init(I2C1, &I2C_InitStructure);

	   I2C_Cmd(I2C1, ENABLE);

	   I2C_AcknowledgeConfig(I2C1, ENABLE);

	   systick_init();

	   sensor_resolution = HTU21_SensorResolution_RH12_TEMP14;
	   humidity_measurement_mode = HTU21_HumidityMeasurementMode_NoHold;
	   temperature_measurement_mode = HTU21_TemperatureMeasurementMode_NoHold;
}

/*******************************************************************************
* Function Name  :	htu21_readTemperature
* Description    : 	Read Temperature, deg.C
* Input          : 	None
* Output         : 	None
* Return         : 	Returns 998 if I2C timed out
* 					Returns 999 if CRC is wrong
* Attention		 : 	Max. measurement time about 50ms.
* 					Accuracy +-0.3deg.C in range 0deg.C - 60deg.C
*******************************************************************************/
float htu21_readTemperature(void){
	uint16_t raw_temperature = 0;
	uint8_t crc;

	/* Request a temperature reading */
	while(I2C_GetFlagStatus(HTU21D_I2C, I2C_FLAG_BUSY));	/* Wait while the bus is busy */
	I2C_AcknowledgeConfig(HTU21D_I2C, ENABLE);		/* Enable Acknowledge */
	I2C_GenerateSTART(HTU21D_I2C, ENABLE);		/* Send "Start" condition */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_MODE_SELECT));		/* Test on EV5 and clear it */
	I2C_Send7bitAddress(HTU21D_I2C,  HTDU21D_ADDRESS, I2C_Direction_Transmitter);	/* Send HTU21D address for write */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	/* Test on EV6 and clear it */
	I2C_Cmd(HTU21D_I2C, ENABLE);	/* Clear EV6 by setting again the PE bit */
	I2C_SendData(HTU21D_I2C, temperature_measurement_mode);		/* Send "temperature read" command */
    while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));		/* Test on EV8 and clear it */

    /* Measurement delay */
    switch (sensor_resolution) {
		case HTU21_SensorResolution_RH12_TEMP14:
			systick_delayMs(50);
			break;
		case HTU21_SensorResolution_RH8_TEMP12:
			systick_delayMs(25);
			break;
		case HTU21_SensorResolution_RH10_TEMP13:
			systick_delayMs(13);
			break;
		case HTU21_SensorResolution_RH11_TEMP11:
			systick_delayMs(7);
			break;
	}

    /* Reads MSB byte, LSB byte & Checksum */
	I2C_GenerateSTART(HTU21D_I2C, ENABLE);	/* Send "Start" condition */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_MODE_SELECT));	/* Test on EV8 and clear it */
	I2C_Send7bitAddress(HTU21D_I2C,  HTDU21D_ADDRESS, I2C_Direction_Receiver);	/* Send HTU21D address for read */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));	/* Test on EV7 and clear it */
	raw_temperature = I2C_ReceiveData(HTU21D_I2C);	/* Read a MSB byte from the HTU21D */
	raw_temperature <<= 8;		/* Shift left 8-bit of MSB byte from the HTU21D */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));		/* Test on EV7 and clear it */
	raw_temperature |= I2C_ReceiveData(HTU21D_I2C);		/* Read a LSB byte from the HTU21D */
	I2C_AcknowledgeConfig(HTU21D_I2C, DISABLE);		/* Disable Acknowledge */
	I2C_GenerateSTOP(HTU21D_I2C, ENABLE);		/* Send STOP condition */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));		/* Test on EV7 and clear it */
	crc = I2C_ReceiveData(HTU21D_I2C);		/* Read CRC byte from the HTU21D */
	I2C_AcknowledgeConfig(HTU21D_I2C, ENABLE);		/* Enable Acknowledge */

	/* Check CRC8 */
	if(htu21_checkCRC8(raw_temperature) != crc)
		return 0.00;

	/* Calculate temperature */
	temperature = -46.85 + 0.002681 * (float)raw_temperature;

	return temperature;
}

/*******************************************************************************
* Function Name  :	htu21_readHumidity
* Description    : 	Read Humidity, %RH
* Input          : 	None
* Output         : 	None
* Return         : 	Returns 998 if I2C timed out
* 					Returns 999 if CRC is wrong
* Attention		 : 	Max. measurement time about 16ms.
* 					Accuracy +-2%RH in range 20%RH - 80%RH at 25deg.C only
*******************************************************************************/
float htu21_readHumidity(void){
	uint16_t raw_humidity = 0;
	uint8_t crc;

	/* Request a humidity reading */
	while(I2C_GetFlagStatus(HTU21D_I2C, I2C_FLAG_BUSY));	/* Wait while the bus is busy */
	I2C_AcknowledgeConfig(HTU21D_I2C, ENABLE);	/* Enable Acknowledge */
	I2C_GenerateSTART(HTU21D_I2C, ENABLE);	/* Send "Start" condition */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_MODE_SELECT));	/* Test on EV5 and clear it */
	I2C_Send7bitAddress(HTU21D_I2C,  HTDU21D_ADDRESS, I2C_Direction_Transmitter);	/* Send HTU21D address for write */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	/* Test on EV6 and clear it */
	I2C_Cmd(HTU21D_I2C, ENABLE);	/* Clear EV6 by setting again the PE bit */
	I2C_SendData(HTU21D_I2C, humidity_measurement_mode);	/* Send "measurement read" command */
    while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	/* Test on EV8 and clear it */

    /* Measurement delay */
    switch (sensor_resolution) {
		case HTU21_SensorResolution_RH12_TEMP14:
			systick_delayMs(16);
			break;
		case HTU21_SensorResolution_RH8_TEMP12:
			systick_delayMs(8);
			break;
		case HTU21_SensorResolution_RH10_TEMP13:
			systick_delayMs(5);
			break;
		case HTU21_SensorResolution_RH11_TEMP11:
			systick_delayMs(3);
			break;
	}

    /* Reads MSB byte, LSB byte & Checksum */
	I2C_GenerateSTART(HTU21D_I2C, ENABLE);	/* Send "Start" condition */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_MODE_SELECT));	/* Test on EV8 and clear it */
	I2C_Send7bitAddress(HTU21D_I2C,  HTDU21D_ADDRESS, I2C_Direction_Receiver);	/* Send HTU21D address for read */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));	/* Test on EV7 and clear it */
	raw_humidity = I2C_ReceiveData(HTU21D_I2C);	/* Read a MSB byte from the HTU21D */
	raw_humidity <<= 8;	/* Shift left 8-bit of MSB byte from the HTU21D */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));	/* Test on EV7 and clear it */
	raw_humidity |= I2C_ReceiveData(HTU21D_I2C);	/* Read a LSB byte from the HTU21D */
	I2C_AcknowledgeConfig(HTU21D_I2C, DISABLE);	/* Disable Acknowledge */
	I2C_GenerateSTOP(HTU21D_I2C, ENABLE);	/* Send STOP condition */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));	/* Test on EV7 and clear it */
	crc = I2C_ReceiveData(HTU21D_I2C);	/* Read CRC byte from the HTU21D */
	I2C_AcknowledgeConfig(HTU21D_I2C, ENABLE);	/* Enable Acknowledge */

	/* Check CRC8 */
	if(htu21_checkCRC8(raw_humidity) != crc)
		return 0.00;

	/* clear two last status bits */
	raw_humidity ^= 0x02;

	/* Calculate temperature */
	humidity = -6.00 + 0.001907 * (float)raw_humidity;

	return humidity;
}



/*******************************************************************************
* Function Name  :	htu21_readCompansatedHumidity
* Description    : 	Calculates Compensated Humidity, %RH
* Input          : 	None
* Output         : 	None
* Return         : 	float
* Attention		 : 	Max. measurement time about 70ms.
* 					Accuracy +-2%RH in range 0%RH - 100%RH at tmp. range 0deg.C - 80deg.C
*******************************************************************************/
float htu21_readCompensatedHumidity(void){


	humidity = htu21_readHumidity();
	temperature = htu21_readTemperature();

	if (humidity == 0.00 || temperature == 0.00) {
		return 0.00;
	}

	compensated_humidity = humidity + (25 - temperature) * TEMP_COEFFICIENT;

	return compensated_humidity;
}


/*******************************************************************************
* Function Name  :	htu21_softwareReset
* Description    : 	Reboot the sensor
* Input          : 	None
* Output         : 	None
* Return         : 	None
* Attention		 : 	Switch sensor OFF & ON again. Takes about 15ms.
* 					All registers set to default except heater bit.
*******************************************************************************/
void htu21_softwareReset(void){
	/* Wait to be sure bus is not busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	/* Start "Start" sequence */
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send 7-bit address and transmission direction for read/write operation */
	I2C_Send7bitAddress(I2C1,  HTDU21D_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send "temperature read" command */
	I2C_SendData(I2C1, SOFT_RESET);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(I2C1, ENABLE);
    systick_delayMs(15);
}


/*******************************************************************************
* Function Name  :	htu21_readRegister
* Description    : 	Read register of the HTU21 sensor
* Input          : 	None
* Output         : 	None
* Return         : 	8-bit data
* Attention		 : 	None
*******************************************************************************/
uint8_t htu21_readRegister(void){
	uint8_t user_register;

	/* Wait while the bus is busy */
	while(I2C_GetFlagStatus(HTU21D_I2C, I2C_FLAG_BUSY));

	/* Enable Acknowledge */
	I2C_AcknowledgeConfig(HTU21D_I2C, ENABLE);

	/* Send "Start" condition */
	I2C_GenerateSTART(HTU21D_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send HTU21D address for write */
	I2C_Send7bitAddress(HTU21D_I2C,  HTDU21D_ADDRESS, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(HTU21D_I2C, ENABLE);

	/* Send "Read User Register" command */
	I2C_SendData(HTU21D_I2C, READ_USER_REG);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send "Start" condition */
	I2C_GenerateSTART(HTU21D_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send HTU21D address for read */
	I2C_Send7bitAddress(HTU21D_I2C,  HTDU21D_ADDRESS, I2C_Direction_Receiver);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	/* Disable Acknowledgement */
	I2C_AcknowledgeConfig(HTU21D_I2C, DISABLE);
	/* Send STOP Condition */
	I2C_GenerateSTOP(HTU21D_I2C, ENABLE);
	/* Test on EV7 and clear it */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
	/* Read a byte from the HTU21D */
	user_register = I2C_ReceiveData(HTU21D_I2C);

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(HTU21D_I2C, ENABLE);

	return user_register;
}


/*******************************************************************************
* Function Name  :	htu21_writeRegister
* Description    : 	Write 8-bit data to register of the HTU21 sensor
* Input          : 	8-bit data
* Output         : 	None
* Return         : 	None
* Attention		 : 	None
*******************************************************************************/
void htu21_writeRegister(uint8_t data){
	/* Wait while the bus is busy */
	while(I2C_GetFlagStatus(HTU21D_I2C, I2C_FLAG_BUSY));

	/* Enable Acknowledge */
	I2C_AcknowledgeConfig(HTU21D_I2C, ENABLE);

	/* Send "Start" condition */
	I2C_GenerateSTART(HTU21D_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send HTU21D address for write */
	I2C_Send7bitAddress(HTU21D_I2C,  HTDU21D_ADDRESS, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(HTU21D_I2C, ENABLE);

	/* Send "Read User Register" command */
	I2C_SendData(HTU21D_I2C, WRITE_USER_REG);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send "data" */
	I2C_SendData(HTU21D_I2C, data);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(HTU21D_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP Condition */
	I2C_GenerateSTOP(HTU21D_I2C, ENABLE);
}


/*******************************************************************************
* Function Name  :	htu21_setResolution
* Description    : 	Set HTU21 resolution
* Input          : 	8-bit data
* Output         : 	None
* Return         : 	None
* Attention		 : 	None
*******************************************************************************/
void htu21_setResolution(HTU21SensorResolution_TypeDef resolution){
	uint8_t user_register;

	/* Get the current register state */
	user_register = htu21_readRegister();

	/* Replace current resolution bits with "0" */
	user_register &= 0x7E;

	/* Add new resolution bits to userRegisterData */
	user_register |= resolution;

	/* Write updated userRegisterData to sensor */
	htu21_writeRegister(user_register);
}



/*******************************************************************************
* Function Name  :	htu21_checkBatteryStatus
* Description    : 	Check battery status
* Input          : 	None
* Output         : 	None
* Return         : 	HTU21_BatteryStatus_TypeDef
* Attention		 : 	Returns true if VDD > 2.25V
* 					Returns false if VDD < 2.25V
*******************************************************************************/
HTU21_BatteryStatus_TypeDef htu21_checkBatteryStatus(void){
	uint8_t user_register;

	/* Read user register */
	user_register = htu21_readRegister();

	/* Mask 6th bit */
	user_register &= 0x40;

	/* Check 6th bit. if set return false */
	if(user_register == 00)
		return HTU21_BatteryStatus_BatteryOk;

	return HTU21_BatteryStatus_EndOfBattery;
}


/*******************************************************************************
* Function Name  :	htu21_enableHeater
* Description    : 	Turn ON/OFF build-in Heater
* Input          : 	HTU21HeaterSwitch_TypeDef
* Output         : 	None
* Return         : 	None
* Attention		 : 	The heater consumtion 5.5mW. Temperature increase of 0.5-1.5 deg.C
* 					Relative humidity drops upon rising temperature
* 					Mostly used for diagnostic of sensor's functionality
*******************************************************************************/
void htu21_enableHeater(HTU21HeaterSwitch_TypeDef value){
	uint8_t user_register;

	/* Get the current user register state */
	user_register = htu21_readRegister();

	switch (value) {
		case HTU21_HeaterSwitch_On:
			user_register |= value;
			break;
		case HTU21_HeaterSwitch_Off:
			user_register &= value;
			break;
	}

	/* Write updated user register state */
	htu21_writeRegister(user_register);
}

/*******************************************************************************
* Function Name  :	htu21_checkCRC8
* Description    : 	Calculates Data CRC8 for 16bit received Data
* Input          : 	16-bit data
* Output         : 	None
* Return         : 	8-bit data
* Attention		 : 	None
*******************************************************************************/
uint8_t htu21_checkCRC8(uint16_t data){
	uint8_t bit;

	for (bit = 0; bit < 16; bit++)
	{
		if (data & 0x8000)
		{
			data = (data << 1) ^ CRC8_POLYNOMINAL;
		}
		else
		{
			data <<= 1;
		}
	}
	data >>= 8;

	return data;
}
