/*********************************************************************************************************
* File  		:	stm32f10x_fatfs.h
* Description	: 	STM32F10x library for micro-sd card
* Author		: 	Aytac Dilek
* Note			: 	GNU General Public License, version 3 (GPL-3.0)
*********************************************************************************************************/
#ifndef __STM32F10X_FATFS_H
#define __STM32F10X_FATFS_H



/* Includes *********************************************************************************************/
#include "stm32f10x.h"
#include "ff.h"
#include "stm32f10x_microsd.h"
//#include "stm32f10x_usbdisk.h"
//#include "stm32f10x_atadrive.h"



/* Enumerations *****************************************************************************************/
typedef enum{
	/* Media types (in this LIB only MMC) */
	MMC_0 = 0		// in this LIB is only "MMC" supported
}MEDIA_TypeDef;

typedef enum {
	/* Error Codes */
	FATFS_OK =0,
	FATFS_NO_MEDIA,
	FATFS_MOUNT_ERR,
	FATFS_GETFREE_ERR,
	FATFS_UNLINK_ERR,
	FATFS_OPEN_ERR,
	FATFS_CLOSE_ERR,
	FATFS_PUTS_ERR,
	FATFS_SEEK_ERR,
	FATFS_RD_STRING_ERR,
	FATFS_RD_BLOCK_ERR,
	FATFS_WR_BLOCK_ERR,
	FATFS_EOF,
	FATFS_DISK_FULL
}FATFS_t;

typedef enum {
	/* Modes for OpenFile */
	F_RD =0,    // Open for reading (only if file exists)
	F_WR,       // open it for writing (only if file exists) and append data
	F_WR_NEW,   // open it for writing (and event. edit) and append data
	F_WR_CLEAR  // open for writing (Delete old files)
}FMODE_t;



/* Gloabal Functions ************************************************************************************/
void fatfs_init(void);
FATFS_t fatfs_checkMedia(MEDIA_TypeDef dev);
FATFS_t UB_Fatfs_Mount(MEDIA_TypeDef dev);
FATFS_t UB_Fatfs_UnMount(MEDIA_TypeDef dev);
FATFS_t UB_Fatfs_DelFile(const char* name);
FATFS_t UB_Fatfs_OpenFile(FIL* fp, const char* name, FMODE_t mode);
FATFS_t UB_Fatfs_CloseFile(FIL* fp);
FATFS_t UB_Fatfs_WriteString(FIL* fp, const char* text);
FATFS_t UB_Fatfs_ReadString(FIL* fp, char* text, uint32_t len);
uint32_t UB_Fatfs_FileSize(FIL* fp);
FATFS_t UB_Fatfs_ReadBlock(FIL* fp, unsigned char* buf, uint32_t len, uint32_t* read);
FATFS_t UB_Fatfs_WriteBlock(FIL* fp, unsigned char* buf, uint32_t len, uint32_t* write);



#endif // __STM32F10X_FATFS_H
