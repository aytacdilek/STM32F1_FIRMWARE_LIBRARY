/*
 * File			:	stm32f10x_hmc5883l.c
 * Description		:	STM32F10x library for HMC5883 digital compass sensor
 * Datum		:	2015.03.20
 * Version		:	1.0
 * Author		:	Aytac Dilek
 * email		:	aytacdilek@gmail.com
 * Web			:
 * Platform		:	OPEN103Z-B
 * CPU			:	STM32F103ZET6
 * IDE			:	CooCox CoIDE 1.7.7
 * GCC			:	4.8 2014q2
 * Module		:	HMC5883L
 * Function		:	Compassing
 */


/* Includes */
#include "stm32f10x_hmc5883l.h"



/*******************************************************************************
* Function Name  : hmc5883l_init
* Description    : Initialize HMC5883L compass sensor
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void hmc5883l_init(void){
	systick_init();
	hmc5883l_i2c_init();
	hmc5883l_registerInit();
}



/*******************************************************************************
* Function Name  : hmc5883l_registerInit
* Description    : Initialize register of HMC5883L magnetometer sensor
* Input          : None
* Output         : None
* Return         : None
* Attention		 : Internal Function
*******************************************************************************/
void hmc5883l_registerInit(void){
	HMC5883L_InitStructure.HMC5883L_MeasurementBias					=	HMC5883L_MEASUREMENT_BIAS_NORMAL;
	HMC5883L_InitStructure.HMC5883L_DataOutputRate					=	HMC5883L_DATA_OUTPUT_RATE_15;
	HMC5883L_InitStructure.HMC5883L_AveragedSamplePerMeasurement	=	HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_1;
	HMC5883L_InitStructure.HMC5883L_Gain							= 	HMC5883L_GAIN_1090;
	HMC5883L_InitStructure.HMC5883L_OperatingMode					=	HMC5883L_MEASUREMENT_MODE_CONTINOUS;

	hmc5883l_setMeasurementBias(HMC5883L_InitStructure.HMC5883L_MeasurementBias);
	hmc5883l_setDataOutputRate(HMC5883L_InitStructure.HMC5883L_DataOutputRate);
	hmc5883l_setAveragedSampleNumber(HMC5883L_InitStructure.HMC5883L_AveragedSamplePerMeasurement);
	hmc5883l_setGain(HMC5883L_InitStructure.HMC5883L_Gain);
	hmc5883l_setOperatingMode(HMC5883L_InitStructure.HMC5883L_OperatingMode);
}


/*******************************************************************************
* Function Name  : hmc5883l_i2c_init
* Description    : Initialize i2c peripheral of stm32f1 microcontroller
* Input          : None
* Output         : None
* Return         : None
* Attention		 : Internal Function
*******************************************************************************/
void hmc5883l_i2c_init(void){
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable Clocks */
	RCC_APB1PeriphClockCmd(HMC5883L_RCC_I2C, ENABLE);
	RCC_APB2PeriphClockCmd(HMC5883L_RCC_PORT, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Configure I2C1 pins: PB6->SCL and PB7->SDA */
	GPIO_InitStructure.GPIO_Pin =  HMC5883L_I2C_SCL_PIN | HMC5883L_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(HMC5883L_I2C_PORT, &GPIO_InitStructure);

	/* Configure I2C peripheral */
	I2C_DeInit(I2C1);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = HMC5883L_I2C_SPEED;
	I2C_Init(HMC5883L_I2C, &I2C_InitStructure);

	I2C_Cmd(HMC5883L_I2C, ENABLE);

	I2C_AcknowledgeConfig(HMC5883L_I2C, ENABLE);
}



/*******************************************************************************
* Function Name  :	hmc5883l_i2c_writeRegister
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_i2c_writeRegister(uint8_t register_address, uint8_t data){
	/* Send "Start" condition */
	I2C_GenerateSTART(HMC5883L_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send HTU21D address for write */
	I2C_Send7bitAddress(HMC5883L_I2C,  HMC5883L_I2C_ADDRESS, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(HMC5883L_I2C, ENABLE);

	/* Send register address of the HMC5883 to write 8-bit data */
	I2C_SendData(HMC5883L_I2C, register_address);

	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send "data" */
	I2C_SendData(HMC5883L_I2C, data);

	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP Condition */
	I2C_GenerateSTOP(HMC5883L_I2C, ENABLE);
}



/*******************************************************************************
* Function Name  :	hmc5883l_i2c_readRegister
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_i2c_readRegister(uint8_t register_address, uint8_t *buffer, uint8_t buffer_size){
	/* Wait while the bus is busy */
	while(I2C_GetFlagStatus(HMC5883L_I2C, I2C_FLAG_BUSY));

	/* Send "Start" condition */
	I2C_GenerateSTART(HMC5883L_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send HMC5883L address for write */
	I2C_Send7bitAddress(HMC5883L_I2C,  HMC5883L_I2C_ADDRESS, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(HMC5883L_I2C, ENABLE);

	/* Send Register address to read */
	I2C_SendData(HMC5883L_I2C, register_address);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send "Start" condition */
	I2C_GenerateSTART(HMC5883L_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send HMC5883L address for write */
	I2C_Send7bitAddress(HMC5883L_I2C,  HMC5883L_I2C_ADDRESS, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	 /* While there is data to be read */
	while (buffer_size)
	{
		if (buffer_size == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(HMC5883L_I2C, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(HMC5883L_I2C, ENABLE);
		}
		/* Test on EV7 and clear it */
		if (I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* Read a byte from the HMC5883L */
			*buffer = I2C_ReceiveData(HMC5883L_I2C);
			/* Point to the next location where the byte read will be saved */
			buffer++;
			/* Decrement the read bytes counter */
			buffer_size--;
		}
	}

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(HMC5883L_I2C, ENABLE);
}



/*******************************************************************************
* Function Name  :	hmc5883l_checkdIdentificationRegister
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
bool hmc5883l_checkdIdentificationRegister(){
	uint8_t tmp[3] = {0};

	hmc5883l_i2c_readRegister(HMC5883L_I2C_IRA, tmp, sizeof(tmp));

	return (tmp[0] == 'H' && tmp[1] == '4' && tmp[2] == '3') ? true : false;
}



/*******************************************************************************
* Function Name  :	hmc5883l_getAveragedSampleNumber
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	None
* Output         :	None
* Return         :	AVERAGED_SAMPLE_NUMBER_TypeDef, number of samples averaged per measurement
* Attention		 :	None
*******************************************************************************/
HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_TypeDef hmc5883l_getAveragedSampleNumber(void){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_CRA, &temp, sizeof(temp));

	/* Mask CRA5 and CRA6 bits */
	temp = temp & 0x60;

	if (temp == HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_1) {
		return HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_1;
	}
	else if(temp == HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_2){
		return HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_2;
	}
	else if(temp == HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_4){
		return HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_4;
	}
	else
		return HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_8;
}




/*******************************************************************************
* Function Name  :	hmc5883l_setAveragedSampleNumber
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_setAveragedSampleNumber(HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_TypeDef sample_number){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_CRA, &temp, sizeof(temp));

	/* Clear CRA5 and CRA6 bits */
	temp = temp & 0x9F;
	/* Set CRA5 and CRA6 bits according to chosen configuration */
	temp = temp | sample_number;

	/* Write updated value to configuration register A */
	hmc5883l_i2c_writeRegister(HMC5883L_I2C_CRA, temp);
}




/*******************************************************************************
* Function Name  :	hmc5883l_getDataOutputRate
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	None
* Output         :	None
* Return         :	DATA_OUTPUT_RATE_TypeDef, data output rate in HZ
* Attention		 :	None
*******************************************************************************/
HMC5883L_DATA_OUTPUT_RATE_TypeDef hmc5883l_getDataOutputRate(void){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_CRA, &temp, sizeof(temp));

	/* Mask CRA5 and CRA6 bits */
	temp = temp & 0x1E;

	if (temp == HMC5883L_DATA_OUTPUT_RATE_0_75) {
		return HMC5883L_DATA_OUTPUT_RATE_0_75;
	}
	else if(temp == HMC5883L_DATA_OUTPUT_RATE_1_5){
		return HMC5883L_DATA_OUTPUT_RATE_1_5;
	}
	else if(temp == HMC5883L_DATA_OUTPUT_RATE_3){
		return HMC5883L_DATA_OUTPUT_RATE_3;
	}
	else if(temp == HMC5883L_DATA_OUTPUT_RATE_7_5){
		return HMC5883L_DATA_OUTPUT_RATE_7_5;
	}
	else if(temp == HMC5883L_DATA_OUTPUT_RATE_15){
		return HMC5883L_DATA_OUTPUT_RATE_15;
	}
	else if(temp == HMC5883L_DATA_OUTPUT_RATE_30){
		return HMC5883L_DATA_OUTPUT_RATE_30;
	}
	else
		return HMC5883L_DATA_OUTPUT_RATE_75;
}




/*******************************************************************************
* Function Name  :	hmc5883l_setAveragedSampleNumber
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_setDataOutputRate(HMC5883L_DATA_OUTPUT_RATE_TypeDef output_rate){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_CRA, &temp, sizeof(temp));

	/* Clear CRA5 and CRA6 bits */
	temp = temp & 0xE3;
	/* Set CRA5 and CRA6 bits according to chosen configuration */
	temp = temp | output_rate;

	/* Write updated value to configuration register A */
	hmc5883l_i2c_writeRegister(HMC5883L_I2C_CRA, temp);
}



/*******************************************************************************
* Function Name  :	hmc5883l_getMeasurementBias
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	None
* Output         :	None
* Return         :	DATA_OUTPUT_RATE_TypeDef, data output rate in HZ
* Attention		 :	None
*******************************************************************************/
HMC5883L_MEASUREMENT_BIAS_TypeDef hmc5883l_getMeasurementBias(void){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_CRA, &temp, sizeof(temp));

	/* Mask MS0 and MS1 bits */
	temp = temp & 0x03;

	if (temp == HMC5883L_MEASUREMENT_BIAS_NORMAL) {
		return HMC5883L_MEASUREMENT_BIAS_NORMAL;
	}
	else if(temp == HMC5883L_MEASUREMENT_BIAS_POSITIVE){
		return HMC5883L_MEASUREMENT_BIAS_POSITIVE;
	}
	else
		return HMC5883L_MEASUREMENT_BIAS_NEGATIVE;
}




/*******************************************************************************
* Function Name  :	hmc5883l_setMeasurementBias
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_setMeasurementBias(HMC5883L_MEASUREMENT_BIAS_TypeDef bias_value){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_CRA, &temp, sizeof(temp));

	/* Clear CRA5 and CRA6 bits */
	temp = temp & 0xFC;
	/* Set CRA5 and CRA6 bits according to chosen configuration */
	temp = temp | bias_value;

	/* Write updated value to configuration register A */
	hmc5883l_i2c_writeRegister(HMC5883L_I2C_CRA, temp);
}



/*******************************************************************************
* Function Name  :	hmc5883l_setOperatingMode
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_setOperatingMode(HMC5883L_OPERATING_MODE_TypeDef operating_mode){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_MR, &temp, sizeof(temp));

	/* Clear CRA5 and CRA6 bits */
	temp = temp & 0xFC;
	/* Set CRA5 and CRA6 bits according to chosen configuration */
	temp = temp | operating_mode;

	/* Write updated value to configuration register A */
	hmc5883l_i2c_writeRegister(HMC5883L_I2C_MR, temp);
}



/*******************************************************************************
* Function Name  :	hmc5883l_setMeasurementBias
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_setGain(HMC5883L_GAIN_TypeDef gain){
	uint8_t temp;

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_CRB, &temp, sizeof(temp));

	/* Clear CRB7, CRB6, and CRB5 bits */
	temp = temp & 0x1F;
	/* Set CRB7, CRB6, and CRB5 bits according to chosen configuration */
	temp = temp | gain;

	/* Write updated value to configuration register A */
	hmc5883l_i2c_writeRegister(HMC5883L_I2C_MR, temp);
}




/*******************************************************************************
* Function Name  :	hmc5883l_checkLockStatus
* Description    :	Check Data Output Register Lock Status
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
bool hmc5883l_checkLockStatus(void){
	uint8_t temp;

	/* Read current value of status register*/
	hmc5883l_i2c_readRegister(HMC5883L_I2C_SR, &temp, sizeof(temp));

	/* Mask SR1 bit of status register */
	temp = temp & 0x02;

	return temp == 0x02 ? true : false;
}



/*******************************************************************************
* Function Name  :	hmc5883l_checkLockStatus
* Description    :	Check Data Output Register Lock Status
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
bool hmc5883l_checkReadyBitStatus(void){
	uint8_t temp;

	/* Read current value of status register*/
	hmc5883l_i2c_readRegister(HMC5883L_I2C_SR, &temp, sizeof(temp));

	/* Mask SR1 bit of status register */
	temp = temp & 0x01;

	return temp == 0x01 ? true : false;
}



/*******************************************************************************
* Function Name  :	hmc5883l_setMeasurementBias
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_readDataRegisters(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z){
	uint8_t temp[6] = {0};

	/* Read current value of configuration register A */
	hmc5883l_i2c_readRegister(HMC5883L_I2C_DXR_MSB, &temp, sizeof(temp));

	*raw_x = ((int16_t) temp[0] << 8);
	*raw_x = *raw_x | (int16_t) temp[1];

	*raw_z = ((int16_t)temp[2] << 8);
	*raw_z = *raw_z | (int16_t) temp[3];

	*raw_y = ((int16_t)temp[4] << 8);
	*raw_y = *raw_y | (int16_t) temp[5];

	// TODO: Check error
}



/*******************************************************************************
* Function Name  :	hmc5883l_readGauss
* Description    :	Send 8-bit data to register of HMC5883L compass sensor
* Input          : 	register_address, address of the register
* 					data, 8-bit data to write to register
* Output         :	None
* Return         :	None
* Attention		 :	None
*******************************************************************************/
void hmc5883l_readGauss(double *gauss_x, double *gauss_y, double *gauss_z){
	int16_t raw_x, raw_y, raw_z;

	hmc5883l_readDataRegisters(&raw_x, &raw_y, &raw_z);

	/* Convert heading reading to gauss unit */
	uint16_t divisor;
	switch (HMC5883L_InitStructure.HMC5883L_Gain) {
		case HMC5883L_GAIN_1370:
			divisor = 1370;
			break;
		case HMC5883L_GAIN_1090:
			divisor = 1090;
			break;
		case HMC5883L_GAIN_820:
			divisor = 820;
			break;
		case HMC5883L_GAIN_660:
			divisor = 660;
			break;
		case HMC5883L_GAIN_440:
			divisor = 440;
			break;
		case HMC5883L_GAIN_390:
			divisor = 390;
			break;
		case HMC5883L_GAIN_330:
			divisor = 330;
			break;
		case HMC5883L_GAIN_230:
			divisor = 230;
			break;
	}

	*gauss_x = (double) raw_x / divisor;
	*gauss_y = (double) raw_y / divisor;
	*gauss_z = (double) raw_z / divisor;
}



double hmc5883l_readDegree(void){
	int16_t raw_x, raw_y, raw_z;
	double tan_heading;
	double heading;

	hmc5883l_readDataRegisters(&raw_x, &raw_y, &raw_z);

	tan_heading = atan2(raw_y, raw_x);

	if(tan_heading < 0)
		tan_heading+= 2 * PI;

	heading = tan_heading * 180 / PI;

	return heading;
}
