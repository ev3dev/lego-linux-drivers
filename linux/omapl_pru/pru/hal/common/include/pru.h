/* --------------------------------------------------------------------------
  FILE        : pru.h
  PROJECT     : DA8xx/OMAP-L138/C674x PRU Development
  DESC        : PRU Load and Run API Definitions
-----------------------------------------------------------------------------*/

#ifndef _PRU_H_
#define _PRU_H_

#include <linux/types.h>

#include "csl/cslr_prucore.h"

#define PRU_NUM0		(0)
#define PRU_NUM1		(1)

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/
// PRU Memory Macros
#define PRU0_DATA_RAM_START  (0x01C30000)
#define PRU0_PROG_RAM_START  (0x01C38000)

#define PRU1_DATA_RAM_START  (0x01C32000)
#define PRU1_PROG_RAM_START  (0x01C3C000)

#define PRU_DATA_RAM_SIZE    (0x200)
#define PRU_PROG_RAM_SIZE    (0x1000)

#define PRU_PRU0_BASE_ADDRESS			0

/*
 *====================
 * Typedef structures
 *====================
 */

typedef struct arm_pru_iomap {
	void *pru_dram_io_addr[2];
	void *pru_ctrl_io_addr[2];
	void *pru_iram_io_addr[2];
	void *mcasp_io_addr;
	dma_addr_t pFifoBufferPhysBase;
	void *pFifoBufferVirtBase;
	int arm_to_pru_irq[2];
} arm_pru_iomap;

/***********************************************************
* Global Function Declarations                             *
***********************************************************/

int pru_enable(u8 pruNum, arm_pru_iomap *pru_arm_iomap);
int pru_load(u8 pruNum, u32 * pruCode, u32 codeSizeInWords, arm_pru_iomap *pru_arm_iomap);
int pru_run(u8 pruNum, arm_pru_iomap *pru_arm_iomap);
int pru_disable(u8 pruNum, arm_pru_iomap *pru_arm_iomap);

short pru_ram_write_data(u32 u32offset, u8 * pu8datatowrite, u16 u16wordstowrite,
			 arm_pru_iomap * pru_arm_iomap);
short pru_ram_read_data(u32 u32offset, u8 * pu8datatoread, u16 u16wordstoread,
			arm_pru_iomap * pru_arm_iomap);
short pru_ram_read_data_4byte(unsigned int u32offset,
				  unsigned int *pu32datatoread,
				  short u16wordstoread);
short pru_ram_write_data_4byte(unsigned int u32offset,
				   unsigned int *pu32datatoread,
				   short u16wordstoread);

/***********************************************************
* End file                                                 *
***********************************************************/

#endif				// End _PRU_H_
