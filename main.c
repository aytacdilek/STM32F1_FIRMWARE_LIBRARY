#include "stm32f10x.h"
#include "ff.h"
#include "diskio.h"

#include "open103z_systick.h"

/* Ram buffers
 * BUFFERSIZE should be between 512 and 1024, depending on available ram on efm32
 */
#define BUFFERSIZE      1024
/* Filename to open/write/read from SD-card */
#define TEST_FILENAME    "logfile.txt"

FIL fsrc;					/* File objects */
FATFS Fatfs;				/* File system specific */
FRESULT res;				/* FatFs function common result code */
UINT br, bw;				/* File read/write count */
DSTATUS resCard;			/* SDcard status */
int8_t ramBufferWrite[BUFFERSIZE];	/* Temporary buffer for write file */
int8_t ramBufferRead[BUFFERSIZE];	/* Temporary buffer for read file */
int8_t StringBuffer[] = "Welcome to Embedded Design World !!!";



int main(void)
{
	int16_t i;
	int16_t filecounter;

	SystemInit();

	/*Step1*/
	/*Initialization file buffer write */
	filecounter = sizeof(StringBuffer);

	for(i = 0; i < filecounter ; i++)
		ramBufferWrite[i] = StringBuffer[i];

	MSD_SPI_Configuration();

	/*Step2*/
	/*Detect micro-SD*/
	while(1){

//		MSD_Init();                     /*Initialize MicroSD driver */

		resCard = disk_initialize(0);       /*Check micro-SD card status */

		switch(resCard){
		case STA_NOINIT:                    /* Drive not initialized */
			break;
		case STA_NODISK:                    /* No medium in the drive */
			break;
		case STA_PROTECT:                   /* Write protected */
			break;
		default:
			break;
		}

		if (!resCard) break;                /* Drive initialized. */

		systick_delayMs(1);
	}

	/*Step3*/
	/* Initialize filesystem */
	if (f_mount(0, &Fatfs) != FR_OK)
	{
		/* Error.No micro-SD with FAT32 is present */
		while(1);
	}

	/*Step4*/
	/* Open  the file for write */
	res = f_open(&fsrc, TEST_FILENAME,  FA_WRITE);
	if (res != FR_OK)
	{
		/*  If file does not exist create it*/
		res = f_open(&fsrc, TEST_FILENAME, FA_CREATE_ALWAYS | FA_WRITE );
		if (res != FR_OK)
		{
			/* Error. Cannot create the file */
			while(1);
		}
	}

	/*Step5*/
	/*Set the file write pointer to first location */
	res = f_lseek(&fsrc, 0);
	if (res != FR_OK)
	{
		/* Error. Cannot set the file write pointer */
		while(1);
	}

	/*Step6*/
	/*Write a buffer to file*/
	res = f_write(&fsrc, ramBufferWrite, filecounter, &bw);
	if ((res != FR_OK) || (filecounter != bw))
	{
		/* Error. Cannot write the file */
		while(1);
	}

	/*Step7*/
	/* Close the file */
	f_close(&fsrc);
	if (res != FR_OK)
	{
		/* Error. Cannot close the file */
		while(1);
	}

	/*Step8*/
	/* Open the file for read */
	res = f_open(&fsrc, TEST_FILENAME,  FA_READ);
	if (res != FR_OK)
	{
		/* Error. Cannot create the file */
		while(1);
	}

	/*Step9*/
	/*Set the file read pointer to first location */
	res = f_lseek(&fsrc, 0);
	if (res != FR_OK)
	{
		/* Error. Cannot set the file pointer */
		while(1);
	}

	/*Step10*/
	/* Read some data from file */
	res = f_read(&fsrc, ramBufferRead, filecounter, &br);
	if ((res != FR_OK) || (filecounter != br))
	{
		/* Error. Cannot read the file */
		while(1);
	}

	/*Step11*/
	/* Close the file */
	res = f_close(&fsrc);
	if (res != FR_OK)
	{
		/* Error. Cannot close the file */
		while(1);
	}

	/*Step12*/
	/*Compare ramBufferWrite and ramBufferRead */
	for(i = 0; i < filecounter ; i++)
	{
		if ((ramBufferWrite[i]) != (ramBufferRead[i]))
		{
			/* Error compare buffers*/
			while(1);
		}
	}

	/*Set here a breakpoint*/
	/*If the breakpoint is trap here then write and read functions were passed */
	while (1)
	{
	}
}
