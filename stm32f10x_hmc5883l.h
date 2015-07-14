/*******************************************************************************
* File  		:	stm32f10x_hmc5883l.h
* Description	: 	STM32F10x library for HMC5883L 3-Axis Digital Compass
* Author		: 	Aytac Dilek
* Note			: 	GNU General Public License, version 3 (GPL-3.0)
*******************************************************************************/

#ifndef __STM32F10X_HMC5883L_H
#define __STM32F10X_HMC5883L_H



/* Includes */
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"

#include "stdbool.h"
#include "math.h"

#include "open103z_config.h"
#include "open103z_systick.h"



/* Defines */
#define HMC5883L_I2C						I2C1
#define HMC5883L_RCC_I2C					RCC_APB1Periph_I2C1
#define HMC5883L_RCC_PORT					RCC_APB2Periph_GPIOB
#define HMC5883L_I2C_PORT					GPIOB
#define HMC5883L_I2C_SCL_PIN				GPIO_Pin_6
#define HMC5883L_I2C_SDA_PIN				GPIO_Pin_7
#define HMC5883L_I2C_SPEED					100000


#define HMC5883L_I2C_ADDRESS 				0x3C	/* Shifted 8-bit I2C address for the sensor */
#define HMC5883L_I2C_CRA					0x00	/* Configuration Register A */
#define HMC5883L_I2C_CRB					0x01	/* Configuration Register B */
#define HMC5883L_I2C_MR						0x02	/* Mode Register */
#define HMC5883L_I2C_DXR_MSB				0x03	/* Data Output X MSB Register */
#define HMC5883L_I2C_DXR_LSB				0x04	/* Data Output X LSB Register */
#define HMC5883L_I2C_DZR_MSB				0x05	/* Data Output Z MSB Register */
#define HMC5883L_I2C_DZR_LSB				0x06	/* Data Output Z LSB Register */
#define HMC5883L_I2C_DYR_MSB				0x07	/* Data Output Y MSB Register */
#define HMC5883L_I2C_DYR_LSB				0x08	/* Data Output Y LSB Register */
#define HMC5883L_I2C_SR						0x09	/* Status Register */
#define HMC5883L_I2C_IRA					0x0A	/* Identification Register A */
#define HMC5883L_I2C_IRB					0x0B	/* Identification Register B */
#define HMC5883L_I2C_IRC					0x0C	/* Identification Register C */

#define PI	3.14159265



/* Enumarations */
/** Measurement Bias Mode Bit of Configuration Register A*/
typedef enum{
	HMC5883L_MEASUREMENT_BIAS_NORMAL = 0x00,					// Default
	HMC5883L_MEASUREMENT_BIAS_POSITIVE = 0x01,
	HMC5883L_MEASUREMENT_BIAS_NEGATIVE = 0x02,
}HMC5883L_MEASUREMENT_BIAS_TypeDef;

/** Data Output Rate Bit of Configuration Register A */
typedef enum{
	HMC5883L_DATA_OUTPUT_RATE_0_75 = 0x00,
	HMC5883L_DATA_OUTPUT_RATE_1_5 = 0x04,
	HMC5883L_DATA_OUTPUT_RATE_3 = 0x08,
	HMC5883L_DATA_OUTPUT_RATE_7_5 = 0x0C,
	HMC5883L_DATA_OUTPUT_RATE_15 = 0x10,			// Default
	HMC5883L_DATA_OUTPUT_RATE_30 = 0x14,
	HMC5883L_DATA_OUTPUT_RATE_75 = 0x18
}HMC5883L_DATA_OUTPUT_RATE_TypeDef;

/** Number of Samples Averaged Bit of Configuration Register A */
typedef enum{
	HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_1 = 0x00,		// Default
	HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_2 = 0x20,
	HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_4 = 0x40,
	HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_8 = 0x30
}HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_TypeDef;

/** Gain Bit of Configuration Register B */
typedef enum{
	HMC5883L_GAIN_1370	=	0b00000000,
	HMC5883L_GAIN_1090	=	0b00100000,		// Default
	HMC5883L_GAIN_820	=	0b01000000,
	HMC5883L_GAIN_660	=	0b01100000,
	HMC5883L_GAIN_440	=	0b10000000,
	HMC5883L_GAIN_390	=	0b10100000,
	HMC5883L_GAIN_330	=	0b11000000,
	HMC5883L_GAIN_230	=	0b11100000,
}HMC5883L_GAIN_TypeDef;

typedef enum{
	HMC5883L_MEASUREMENT_MODE_CONTINOUS = 0,
	HMC5883L_MEASUREMENT_MODE_SINGLE = 1,		// Default
	HMC5883L_MEASUREMENT_MODE_IDLE = 2
}HMC5883L_OPERATING_MODE_TypeDef;

typedef struct{
	HMC5883L_MEASUREMENT_BIAS_TypeDef HMC5883L_MeasurementBias;
	HMC5883L_DATA_OUTPUT_RATE_TypeDef HMC5883L_DataOutputRate;
	HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_TypeDef HMC5883L_AveragedSamplePerMeasurement;
	HMC5883L_GAIN_TypeDef HMC5883L_Gain;
	HMC5883L_OPERATING_MODE_TypeDef HMC5883L_OperatingMode;
}HMC5883L_InitTypeDef;



/* Global Variables */
HMC5883L_InitTypeDef HMC5883L_InitStructure;


/* Global Functions */
void hmc5883l_init(void);
HMC5883L_MEASUREMENT_BIAS_TypeDef hmc5883l_getMeasurementBias(void);
void hmc5883l_setMeasurementBias(HMC5883L_MEASUREMENT_BIAS_TypeDef bias_value);
HMC5883L_DATA_OUTPUT_RATE_TypeDef hmc5883l_getDataOutputRate(void);
void hmc5883l_setDataOutputRate(HMC5883L_DATA_OUTPUT_RATE_TypeDef output_rate);
HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_TypeDef hmc5883l_getAveragedSampleNumber(void);
void hmc5883l_setAveragedSampleNumber(HMC5883L_AVERAGED_SAMPLE_PER_MEASUREMENT_TypeDef sample_number);
void hmc5883l_setGain(HMC5883L_GAIN_TypeDef gain);
void hmc5883l_setOperatingMode(HMC5883L_OPERATING_MODE_TypeDef operating_mode);
bool hmc5883l_checkLockStatus(void);
bool hmc5883l_checkReadyBitStatus(void);
bool hmc5883l_checkdIdentificationRegister();
void hmc5883l_readDataRegisters(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);
void hmc5883l_readGauss(double *gauss_x, double *gauss_y, double *gauss_z);
double hmc5883l_readDegree(void);

#endif // __STM32F10X_HMC5883L_H
