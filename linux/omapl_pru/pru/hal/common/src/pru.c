/*
 * pru/hal/uart/src/pru.c
 *
 * Copyright (C) 2010 Texas Instruments Incorporated
 * Author: Jitendra Kumar <jitendra@mistralsolutions.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

/************************************************************
* Include Files                                             *
************************************************************/

#include <linux/errno.h>
#include <linux/types.h>

#include "csl/cslr.h"
#include "csl/soc_OMAPL138.h"
#include "csl/cslr_prucore.h"
#include "pru.h"

/************************************************************
* Local Function Declarations                               *
************************************************************/

// Load the specified PRU with code
int pru_load(u8 pruNum, u32 * pruCode, u32 codeSizeInWords,
		arm_pru_iomap * pru_arm_iomap)
{
	u32 *pruIram;
	u32 i;

	if (pruNum > CSL_PRUCORE_1)
		return -EINVAL;

	pruIram = pru_arm_iomap->pru_iram_io_addr[pruNum];

	pru_enable(pruNum, pru_arm_iomap);

	// Copy dMAX code to its instruction RAM
	for (i = 0; i < codeSizeInWords; i++) {
		pruIram[i] = pruCode[i];
	}

	return 0;
}

int pru_run(u8 pruNum, arm_pru_iomap * pru_arm_iomap)
{
	CSL_PrucoreRegsOvly hPru;

	if (pruNum > CSL_PRUCORE_1)
		return -EINVAL;

	hPru = pru_arm_iomap->pru_ctrl_io_addr[pruNum];

	// Enable dMAX, let it execute the code we just copied
	CSL_FINST(hPru->CONTROL, PRUCORE_CONTROL_COUNTENABLE, ENABLE);
	CSL_FINST(hPru->CONTROL, PRUCORE_CONTROL_ENABLE, ENABLE);

	return 0;
}

int pru_disable(u8 pruNum, arm_pru_iomap * pru_arm_iomap)
{
	CSL_PrucoreRegsOvly hPru;
	unsigned int delay_cnt;

	if (pruNum > CSL_PRUCORE_1)
		return -EINVAL;

	hPru = pru_arm_iomap->pru_ctrl_io_addr[pruNum];

	CSL_FINST(hPru->CONTROL, PRUCORE_CONTROL_COUNTENABLE, DISABLE);

	for (delay_cnt = 0x10000; delay_cnt > 0; delay_cnt--)
		CSL_FINST(hPru->CONTROL, PRUCORE_CONTROL_ENABLE, DISABLE);

	for (delay_cnt = 0x10000; delay_cnt > 0; delay_cnt--)
		hPru->CONTROL = CSL_PRUCORE_CONTROL_RESETVAL;

	return 0;
}

int pru_enable(u8 pruNum, arm_pru_iomap * pru_arm_iomap)
{
	CSL_PrucoreRegsOvly hPru;

	if (pruNum > CSL_PRUCORE_1)
		return -EINVAL;

	hPru = pru_arm_iomap->pru_ctrl_io_addr[pruNum];
	hPru->CONTROL = CSL_PRUCORE_CONTROL_RESETVAL;

	return 0;
}

/***********************************************************
* End file                                                 *
***********************************************************/

/** ********************************************************************************************************
 * \brief    pru_ram_write_data()       Download the data into data RAM of PRU0 or PRU1 of OMAP L138.
 *
 * This API will be called by the Application to download the data into data RAM of PRU0 or PRU1
 *
 * \param   u32offset           Offset of the data RAM where the data has to be written
 * \param   pu32datatowrite     Pointer to a buffer that holds the data to be written into RAM
* \param    u16wordstowrite     Number of bytes to be written into that RAM
 *
 * \return   SUCCESS or FAILURE
 *
 ***********************************************************************************************************/
short pru_ram_write_data
    (u32 u32offset,
     u8 * pu8datatowrite,
     u16 u16bytestowrite, arm_pru_iomap * pru_arm_iomap) {
	u8 *pu8addresstowrite;
	u16 u16loop;
	u32offset = (unsigned int)pru_arm_iomap->pru_dram_io_addr[0] + u32offset;
	pu8addresstowrite = (u8 *) (u32offset);

	for (u16loop = 0; u16loop < u16bytestowrite; u16loop++)
		*pu8addresstowrite++ = *pu8datatowrite++;
	return 0;
}

/** ********************************************************************************************************
 * \brief    pru_ram_read_data()        Download the data into data RAM of PRU0 or PRU1 of OMAP L138.
 *
 * This API will be called by the Application to read the data from data RAM of PRU0 or PRU1
 *
 * \param   u32offset           Offset of the data RAM where the data has to be read
 * \param   pu8datatoread      Pointer to a buffer that would hold the data to be read from the RAM
* \param    u16bytestoread      Number of bytes to be read from RAM
 *
 * \return   SUCCESS or FAILURE
 *
 ***********************************************************************************************************/
short pru_ram_read_data
    (u32 u32offset,
     u8 * pu8datatoread,
     u16 u16bytestoread, arm_pru_iomap * pru_arm_iomap) {

	u8 *pu8addresstoread;
	u16 u16loop;
	u32offset = (unsigned int)pru_arm_iomap->pru_dram_io_addr[0] + u32offset;
	pu8addresstoread = (u8 *) (u32offset);

	for (u16loop = 0; u16loop < u16bytestoread; u16loop++)
		*pu8datatoread++ = *pu8addresstoread++;

	return 0;
}

/** ********************************************************************************************************
 * \brief    pru_ram_write_data_4byte()		Download the data into data RAM of PRU0 or PRU1 of OMAP L138.
 *
 * This API will be called by the Application to download the data into data RAM of PRU0 or PRU1
 *
 * \param 	u32offset			Offset of the data RAM where the data has to be written
 * \param	pu32datatowrite		Pointer to a buffer that holds the data to be written into RAM
* \param	u16wordstowrite		Number of words to be written into that RAM
 *
 * \r//eturn   SUCCESS or FAILURE
 *
 ***********************************************************************************************************/
short pru_ram_write_data_4byte(unsigned int u32offset,
			       unsigned int *pu32datatowrite,
			       short u16wordstowrite)
{

	unsigned int *pu32addresstowrite;
	short u16loop;

	pu32addresstowrite = (unsigned int *)(u32offset);

	for (u16loop = 0; u16loop < u16wordstowrite; u16loop++)
		*pu32addresstowrite++ = *pu32datatowrite++;

	return 0;
}

/** ********************************************************************************************************
 * \brief    pru_ram_read_data_4byte()		Download the data into data RAM of PRU0 or PRU1 of OMAP L138.
 *
 * This API will be called by the Application to read the data from data RAM of PRU0 or PRU1
 *
 * \param 	u32offset			Offset of the data RAM where the data has to be read
 * \param	pu32datatoread		Pointer to a buffer that would hold the data to be read from the RAM
* \param	u16wordstoread		Number of words to be read from RAM
 *
 * \return   SUCCESS or FAILURE
 *
 ***********************************************************************************************************/
short pru_ram_read_data_4byte(unsigned int u32offset,
			      unsigned int *pu32datatoread,
			      short u16wordstoread)
{
	unsigned int *pu32addresstoread;
	short u16loop;

	pu32addresstoread = (unsigned int *)(u32offset);

	for (u16loop = 0; u16loop < u16wordstoread; u16loop++)
		*pu32datatoread++ = *pu32addresstoread++;

	return 0;
}

