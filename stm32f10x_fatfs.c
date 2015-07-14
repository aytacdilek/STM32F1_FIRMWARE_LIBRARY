/*********************************************************************************************************
 * File				:	stm32f10x_ina219.c
 * Description		:	STM32F10x library for INA219 sensor which reads
 * 						shunt, current and power ratings
 * Datum			:	2015.03.06
 * Version			:	1.0
 * Author			:	Aytac Dilek
 * email			:	aytacdilek@gmail.com
 * Web				:
 * Platform			:	OPEN103Z-B
 * CPU				:	STM32F103ZET6
 * IDE				:	CooCox CoIDE 1.7.7
 * GCC				:	4.8 2014q2
 * Module			:	INA219
 * Function			:	Read shunt, current and power readings
 * Pin Definitions	:	PB6 => SCL
 * 						PB7 => SDA
*********************************************************************************************************/



/* Includes *********************************************************************************************/
#include "stm32f10x_fatfs.h"



/* Global Variables *************************************************************************************/
static FATFS FileSystemObject;



/*********************************************************************************************************
* Function Name		:	fatfs_init
* Description		:	Init function
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	init all systems
*********************************************************************************************************/
void fatfs_init(void){
	// init hardware for the SD card functions
	SD_Init();
	// init the hardware for the USB functions
//	UB_USBDisk_Init();
	// init hardware for ATA features
//	UB_ATADrive_Init();
}



/*********************************************************************************************************
* Function Name		:	fatfs_checkMedia
* Description		:	Query state of the medium
* Input				:	[MMC_0]
* Output			:	None
* Return			:	Media Status
* 						- FATFS_OK    		=>	Media Inserted
* 						- FATFS_NO_MEDIA 	=>	No Media Inserted
* Attention			:	init all systems
*********************************************************************************************************/
FATFS_t fatfs_checkMedia(MEDIA_TypeDef dev){
	FATFS_t ret_wert=FATFS_NO_MEDIA;
	uint8_t check = SD_NOT_PRESENT;

	// Check whether media is inserted
	if(dev == MMC_0)
		check = SD_CheckMedia();
	if(check == SD_PRESENT) {
		ret_wert=FATFS_OK;
	}
	else {
		ret_wert=FATFS_NO_MEDIA;
	}

	return(ret_wert);
}
