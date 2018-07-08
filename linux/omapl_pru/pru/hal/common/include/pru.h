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

#define PRU_INTC_REVID				0
#define PRU_INTC_CONTROL			0x4
#define PRU_INTC_GLBLEN				0x10
#define PRU_INTC_GLBLNSTLVL			0x1C
#define PRU_INTC_STATIDXSET			0x20
#define PRU_INTC_STATIDXCLR			0x24
#define PRU_INTC_ENIDXSET			0x28
#define PRU_INTC_ENIDXCLR			0x2C
#define PRU_INTC_HSTINTENIDXSET			0x34
#define PRU_INTC_HSTINTENIDXCLR			0x38
#define PRU_INTC_GLBLPRIIDX			0x80
#define PRU_INTC_STATSETINT0			0x200
#define PRU_INTC_STATSETINT1			0x204
#define PRU_INTC_STATCLRINT0			0x280
#define PRU_INTC_STATCLRINT1			0x284
#define PRU_INTC_ENABLESET0			0x300
#define PRU_INTC_ENABLESET1			0x304
#define PRU_INTC_ENABLECLR0			0x380
#define PRU_INTC_ENABLECLR1			0x384
#define PRU_INTC_CHANMAP0			0x400
#define PRU_INTC_CHANMAP1			0x404
#define PRU_INTC_CHANMAP2			0x408
#define PRU_INTC_CHANMAP3			0x40C
#define PRU_INTC_CHANMAP4			0x410
#define PRU_INTC_CHANMAP5			0x414
#define PRU_INTC_CHANMAP6			0x418
#define PRU_INTC_CHANMAP7			0x41C
#define PRU_INTC_CHANMAP8			0x420
#define PRU_INTC_CHANMAP9			0x424
#define PRU_INTC_CHANMAP10			0x428
#define PRU_INTC_CHANMAP11			0x42C
#define PRU_INTC_CHANMAP12			0x430
#define PRU_INTC_CHANMAP13			0x434
#define PRU_INTC_CHANMAP14			0x438
#define PRU_INTC_CHANMAP15			0x43C
#define PRU_INTC_HOSTMAP0			0x800
#define PRU_INTC_HOSTMAP1			0x804
#define PRU_INTC_HOSTMAP2			0x808
#define PRU_INTC_HOSTINTPRIIDX0			0x900
#define PRU_INTC_HOSTINTPRIIDX1			0x904
#define PRU_INTC_HOSTINTPRIIDX2			0x908
#define PRU_INTC_HOSTINTPRIIDX3			0x90C
#define PRU_INTC_HOSTINTPRIIDX4			0x910
#define PRU_INTC_HOSTINTPRIIDX5			0x914
#define PRU_INTC_HOSTINTPRIIDX6			0x918
#define PRU_INTC_HOSTINTPRIIDX7			0x91C
#define PRU_INTC_HOSTINTPRIIDX8			0x920
#define PRU_INTC_HOSTINTPRIIDX9			0x924
#define PRU_INTC_POLARITY0			0xD00
#define PRU_INTC_POLARITY1			0xD04
#define PRU_INTC_TYPE0				0xD80
#define PRU_INTC_TYPE1				0xD84
#define PRU_INTC_HOSTINTNSTLVL0			0x1100
#define PRU_INTC_HOSTINTNSTLVL1			0x1104
#define PRU_INTC_HOSTINTNSTLVL2			0x1108
#define PRU_INTC_HOSTINTNSTLVL3			0x110C
#define PRU_INTC_HOSTINTNSTLVL4			0x1110
#define PRU_INTC_HOSTINTNSTLVL5			0x1114
#define PRU_INTC_HOSTINTNSTLVL6			0x1118
#define PRU_INTC_HOSTINTNSTLVL7			0x111C
#define PRU_INTC_HOSTINTNSTLVL8			0x1120
#define PRU_INTC_HOSTINTNSTLVL9			0x1124
#define PRU_INTC_HOSTINTEN			0x1500

/* Macros defining some PRU specific constants. */
#define PRU_INTC_HOSTINTLVL_MAX					9

/*
 *====================
 * Typedef structures
 *====================
 */

typedef struct arm_pru_iomap {
	void *pru_dram_io_addr[2];
	void *pru_ctrl_io_addr[2];
	void *pru_iram_io_addr[2];
	void *pru_intc_io_addr;
	void *mcasp_io_addr;
	void *pFifoBufferPhysBase;
	void *pFifoBufferVirtBase;
	unsigned int   pru_clk_freq;
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
