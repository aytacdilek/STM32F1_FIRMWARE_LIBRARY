/*******************************************************************************
* File  		:	stm32f10x_htu21.h
* Description	: 	STM32F10x library for HTU21 Digital Humidity and Temperature Sensor
* Author		: 	Aytac Dilek
* Note			: 	GNU General Public License, version 3 (GPL-3.0)
*******************************************************************************/

#ifndef __STM32F10X_HTU21_H
#define __STM32F10X_HTU21_H


/* Includes */
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "open103z_config.h"
#include "open103z_systick.h"


/* Defines */
#define HTU21D_I2C	I2C1

#define HTDU21D_ADDRESS 				0x80	/* Shifted 8-bit I2C address for the sensor */

#define WRITE_USER_REG  				0xE6	/* Write user register*/
#define READ_USER_REG  					0xE7	/* Read user register*/
#define SOFT_RESET  					0xFE	/* Soft Reset (takes 15ms). Switch sensor OFF & ON again. All registers set to default */

#define TEMP_COEFFICIENT 				-0.15 	/* Temperature coefficient (from 0deg.C to 80deg.C) */
#define CRC8_POLYNOMINAL 				0x13100 /* CRC8 polynomial for 16bit CRC8 x^8 + x^5 + x^4 + 1 */


/* Enumarations */
typedef enum{
	HTU21_SensorResolution_RH12_TEMP14	= 0x00,	/* RH: 12Bit, measuring time 16ms. Temperature: 14Bit, measuring time 50ms (dafault on Power ON) */
	HTU21_SensorResolution_RH8_TEMP12	= 0x01,	/* RH: 8Bit, measuring time 8ms. Temperature: 12Bit, measuring time 25ms */
	HTU21_SensorResolution_RH10_TEMP13	= 0x80,	/* RH: 10Bit, measuring time 5ms. Temperature: 13Bit, measuring time 13ms. */
	HTU21_SensorResolution_RH11_TEMP11	= 0x81	/* RH: 11Bit, measuring time 3ms. Temperature: 11Bit, measuring time 7ms. */
}HTU21SensorResolution_TypeDef;

typedef enum{
	HTU21_HeaterSwitch_On	= 0x04,
	HTU21_HeaterSwitch_Off	= 0xFB
}HTU21HeaterSwitch_TypeDef;

typedef enum{
	HTU21_HumidityMeasurementMode_Hold		= 0xE5,		/* Trigger Humidity Measurement. Hold master (SCK line is blocked) */
	HTU21_HumidityMeasurementMode_NoHold	= 0xF5		/* Trigger Humidity Measurement. No Hold master (allows other I2C communication on a bus while sensor is measuring) */
}HTU21HumidityMeasurementMode_TypeDef;

typedef enum{
	HTU21_TemperatureMeasurementMode_Hold	= 0xE3,		/* Trigger Temperature Measurement. Hold master (SCK line is blocked) */
	HTU21_TemperatureMeasurementMode_NoHold	= 0xF3		/* Trigger Temperature Measurement. No Hold master (allows other I2C communication on a bus while sensor is measuring) */
}HTU21TemperatureMeasurementMode_TypeDef;

typedef enum{
	HTU21_BatteryStatus_EndOfBattery	= 0x01,			/* VDD < 2.25V */
	HTU21_BatteryStatus_BatteryOk		= 0x00			/* VDD > 2.25V */
}HTU21_BatteryStatus_TypeDef;


/* Global Variables */
float temperature;
float humidity;
float compensated_humidity;
HTU21SensorResolution_TypeDef sensor_resolution;
HTU21HumidityMeasurementMode_TypeDef humidity_measurement_mode;
HTU21TemperatureMeasurementMode_TypeDef temperature_measurement_mode;


/* Global Functions */
void htu21_init(void);
float htu21_readTemperature(void);
float htu21_readHumidity(void);
float htu21_readCompensatedHumidity(void);
void htu21_softwareReset(void);
uint8_t htu21_readRegister(void);
void htu21_writeRegister(uint8_t data);
void htu21_setResolution(HTU21SensorResolution_TypeDef resolution);
HTU21_BatteryStatus_TypeDef htu21_checkBatteryStatus(void);
void htu21_enableHeater(HTU21HeaterSwitch_TypeDef value);
uint8_t htu21_checkCRC8(uint16_t data);


#endif // __STM32F10X_HTU21_H
