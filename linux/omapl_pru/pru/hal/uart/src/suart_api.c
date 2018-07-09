/*
 * pru/hal/uart/src/suart_api.c
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

/*
 *====================
 * Includes
 *====================
 */

#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/types.h>

#include "suart_api.h"
#include "suart_pru_regs.h"
#include "pru.h"
#include "omapl_suart_board.h"
#include "suart_utils.h"
#include "suart_err.h"

#include "csl/cslr_mcasp.h"

static unsigned char gUartStatuTable[8];
static arm_pru_iomap pru_arm_iomap;
static int suart_set_pru_id (unsigned int pru_no);
static void pru_set_rx_tx_mode(u32 pru_mode, u32 pruNum);

#if (PRU_ACTIVE == BOTH_PRU)
void pru_set_ram_data (arm_pru_iomap * arm_iomap_pru)
{

    PRU_SUART_RegsOvly pru_suart_regs = arm_iomap_pru->pru_dram_io_addr[PRU_NUM0];
    unsigned int * pu32SrCtlAddr = (unsigned int *) ((unsigned int)
					arm_iomap_pru->mcasp_io_addr + 0x180);
    pru_suart_tx_cntx_priv * pru_suart_tx_priv = NULL;
    pru_suart_rx_cntx_priv * pru_suart_rx_priv = NULL;
    unsigned char *pu32_pru_ram_base = arm_iomap_pru->pru_dram_io_addr[PRU_NUM0];
	
    /* ***************************** RX PRU - 0  **************************************** */

    /* Chanel 0 context information */
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num =  (0xF & PRU_SUART1_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART1_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART1_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x090); /* SUART1 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART1_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART1_CONFIG_RX_SER << 2));

    /* Chanel 1 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART2_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART2_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART2_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x0B0); /* SUART2 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART2_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART2_CONFIG_RX_SER << 2));

    /* Chanel 2 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART3_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART3_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART3_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x0D0); /* SUART3 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART3_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART3_CONFIG_RX_SER << 2));

    /* Chanel 3 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART4_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART4_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART4_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x0F0); /* SUART4 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART4_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART4_CONFIG_RX_SER << 2));

    /* Chanel 4 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART5_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART5_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART5_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x110); /* SUART5 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART5_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART5_CONFIG_RX_SER << 2));

    /* Chanel 5 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART6_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART6_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART6_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x130); /* SUART6 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART6_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART6_CONFIG_RX_SER << 2));

    /* Chanel 6 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART7_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART7_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART7_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x150); /* SUART7 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART7_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART7_CONFIG_RX_SER << 2));

    /* Chanel 7 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART8_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART8_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART8_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;		
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x170); /* SUART8 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART8_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART8_CONFIG_RX_SER << 2));


    /* ****************************** PRU1 RAM BASE ADDR ******************************** */
    pru_suart_regs = arm_iomap_pru->pru_dram_io_addr[PRU_NUM1];
    pu32_pru_ram_base = arm_iomap_pru->pru_dram_io_addr[PRU_NUM1];

    /* ***************************** TX PRU - 1  **************************************** */
    /* Channel 0 context information */
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART1_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART1_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART1_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;

    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x0B0); /* SUART1 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART1_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART1_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x0090; /* SUART1 TX formatted data base addr */

    /* Channel 1 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART2_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART2_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART2_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;

    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x0DC); /* SUART2 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART2_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART2_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x00BC; /* SUART2 TX formatted data base addr */

    /* Channel 2 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART3_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART3_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART3_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;

    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x108); /* SUART3 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART3_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART3_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x00E8; /* SUART3 TX formatted data base addr */

    /* Channel 3 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART4_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART4_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART4_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;

    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x134); /* SUART4 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART4_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART4_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x0114; /* SUART4 TX formatted data base addr */

    /* Channel 4 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART5_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART5_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART5_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;

    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x160); /* SUART5 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART5_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART5_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x0140; /* SUART5 TX formatted data base addr */

    /* Channel 5 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART6_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART6_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART6_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;

    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x18C); /* SUART6 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART6_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART6_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x016C; /* SUART6 TX formatted data base addr */

    /* Channel 6 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART7_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART7_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART7_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;

    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x1B8); /* SUART7 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART7_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART7_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x0198; /* SUART7 TX formatted data base addr */

    /* Channel 7 context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART8_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART8_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART8_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1;
    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x1E4); /* SUART8 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART8_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART8_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x01C4; /* SUART8  TX formatted data base addr */
}
#else
void pru_set_ram_data (arm_pru_iomap * arm_iomap_pru)
{
    
    PRU_SUART_RegsOvly pru_suart_regs = arm_iomap_pru->pru_dram_io_addr[PRU_NUM0];
    unsigned int * pu32SrCtlAddr = (unsigned int *) ((unsigned int) 
					arm_iomap_pru->mcasp_io_addr + 0x180);	
    pru_suart_tx_cntx_priv * pru_suart_tx_priv = NULL;			
    pru_suart_rx_cntx_priv * pru_suart_rx_priv = NULL;
    unsigned char *pu32_pru_ram_base = arm_iomap_pru->pru_dram_io_addr[PRU_NUM0];
							
    /* ***************************** UART 0  **************************************** */

    /* Channel 0 context information is Tx */
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART1_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART1_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART1_CONFIG_TX_SER)) =  MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1; 
    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x0B0); /* SUART1 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)(MCASP_SRCTL_BASE_ADDR + (PRU_SUART1_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART1_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x0090; /* SUART1 TX formatted data base addr */

    /* Channel 1 is Rx context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART1_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART1_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART1_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x0C0); /* SUART1 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART1_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART1_CONFIG_RX_SER << 2));


    /* ***************************** UART 1  **************************************** */
    /* Channel 2 context information is Tx */
    pru_suart_regs++;    
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART2_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART2_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART2_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1; 
    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x100); /* SUART2 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)(MCASP_SRCTL_BASE_ADDR + (PRU_SUART2_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART2_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x00E0; /* SUART2 TX formatted data base addr */

    /* Channel 3 is Rx context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART2_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART2_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART2_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x110); /* SUART2 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART2_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART2_CONFIG_RX_SER << 2));

    /* ***************************** UART 2  **************************************** */
    /* Channel 4 context information is Tx */
    pru_suart_regs++;    
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART3_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART3_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART3_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1; 
    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x150); /* SUART3 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)( MCASP_SRCTL_BASE_ADDR + (PRU_SUART3_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART3_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x0130; /* SUART3 TX formatted data base addr */

    /* Channel 5 is Rx context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART3_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART3_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART3_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x160); /* SUART3 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART3_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART3_CONFIG_RX_SER << 2));

    /* ***************************** UART 3  **************************************** */
    /* Channel 6 context information is Tx */
    pru_suart_regs++;    
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_TX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART4_CONFIG_TX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART4_CONFIG_DUPLEX & PRU_SUART_HALF_TX_DISABLED) == PRU_SUART_HALF_TX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART4_CONFIG_TX_SER)) = MCASP_SRCTL_TX_MODE;
#endif
    pru_suart_regs->Reserved1 = 1; 
    pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + 0x1A0); /* SUART4 TX context base addr */
    pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)(MCASP_SRCTL_BASE_ADDR + (PRU_SUART4_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(MCASP_XBUF_BASE_ADDR + (PRU_SUART4_CONFIG_TX_SER << 2));
    pru_suart_tx_priv->buff_addr = 0x0180; /* SUART4 TX formatted data base addr */

    /* Channel 7 is Rx context information */
    pru_suart_regs++;
    pru_suart_regs->CH_Ctrl_Config1.mode = SUART_CHN_RX;
    pru_suart_regs->CH_Ctrl_Config1.serializer_num = (0xF & PRU_SUART4_CONFIG_RX_SER);
    pru_suart_regs->CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
    pru_suart_regs->CH_Config2_TXRXStatus.bits_per_char = 8;
#if ((PRU_SUART4_CONFIG_DUPLEX & PRU_SUART_HALF_RX_DISABLED) == PRU_SUART_HALF_RX_DISABLED)
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_DISABLED;
#else
    pru_suart_regs->CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
    *((unsigned int *) (pu32SrCtlAddr + PRU_SUART4_CONFIG_RX_SER)) = MCASP_SRCTL_RX_MODE;
#endif
    /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
    pru_suart_regs->CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
    pru_suart_regs->Reserved1 = 0;
    pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + 0x1B0); /* SUART4 RX context base addr */
    pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(MCASP_RBUF_BASE_ADDR + (PRU_SUART4_CONFIG_RX_SER << 2));
    pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (MCASP_SRCTL_BASE_ADDR + (PRU_SUART4_CONFIG_RX_SER << 2));
}

#endif

/*
 * suart Initialization routine 
 */
short pru_softuart_init(unsigned int txBaudValue,
			unsigned int rxBaudValue,
			unsigned int oversampling,
			const unsigned char *pru_suart_emu_code,
			unsigned int fw_size, arm_pru_iomap * arm_iomap_pru)
{
	unsigned int omapl_addr;
	short status = PRU_SUART_SUCCESS;
	short idx;

	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) && (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		return PRU_SUART_FAILURE;
	}

	pru_arm_iomap = *arm_iomap_pru;

	omapl_addr = (unsigned int)arm_iomap_pru->mcasp_io_addr;
	/* Configure McASP0  */
	suart_mcasp_config(omapl_addr, txBaudValue, rxBaudValue, oversampling,
			   arm_iomap_pru);	

	pru_enable(0, arm_iomap_pru);
#if (!(PRU1_MODE == PRU_MODE_INVALID))
	pru_enable(1, arm_iomap_pru);
#endif

	memset(pru_arm_iomap.pru_dram_io_addr[PRU_NUM0], 0, 512);
#if (!(PRU1_MODE == PRU_MODE_INVALID))
	memset(pru_arm_iomap.pru_dram_io_addr[PRU_NUM1], 0, 512);
#endif

	pru_load(PRU_NUM0, (unsigned int *)pru_suart_emu_code,
		 (fw_size / sizeof(unsigned int)), arm_iomap_pru);
#if (!(PRU1_MODE == PRU_MODE_INVALID))
	pru_load(PRU_NUM1, (unsigned int *)pru_suart_emu_code,
		 (fw_size / sizeof(unsigned int)), arm_iomap_pru);
#endif

	suart_set_pru_id(0);

#if (!(PRU1_MODE == PRU_MODE_INVALID))
	suart_set_pru_id(1);
#endif

	pru_set_rx_tx_mode(PRU0_MODE, PRU_NUM0);
#if (!(PRU1_MODE == PRU_MODE_INVALID))
	pru_set_rx_tx_mode(PRU1_MODE, PRU_NUM1);
#endif
	
	pru_set_ram_data (arm_iomap_pru);

	pru_run(PRU_NUM0, arm_iomap_pru);
#if (!(PRU1_MODE == PRU_MODE_INVALID))
	pru_run(PRU_NUM1, arm_iomap_pru); 
#endif

	/* Initialize gUartStatuTable */
	for (idx = 0; idx < 8; idx++) {
		gUartStatuTable[idx] = ePRU_SUART_UART_FREE;
	}

	return status;
}

static void pru_set_rx_tx_mode(u32 pru_mode, u32 pruNum)
{

	unsigned int pruOffset;

	if (pruNum == PRU_NUM0) 
	{
		/* PRU0 */
		pruOffset = PRU_SUART_PRU0_RX_TX_MODE;
	} 
	else if (pruNum == PRU_NUM1) {
		/* PRU1 */
		pruOffset = PRU_SUART_PRU1_RX_TX_MODE;
	}
	else	
	{
		return;
	}

	pru_ram_write_data(pruOffset, (u8 *)&pru_mode, 1, &pru_arm_iomap);

}


void pru_set_fifo_timeout(u32 timeout)
{
	/* PRU 0 */
	pru_ram_write_data(PRU_SUART_PRU0_IDLE_TIMEOUT_OFFSET, 
				(u8 *)&timeout, 2, &pru_arm_iomap);
#if (!(PRU1_MODE == PRU_MODE_INVALID))
    	/* PRU 1 */
	pru_ram_write_data(PRU_SUART_PRU1_IDLE_TIMEOUT_OFFSET, 
				(u8 *)&timeout, 2, &pru_arm_iomap);
#endif
}

void pru_mcasp_deinit (void)
{
	suart_mcasp_reset (&pru_arm_iomap);
}

short pru_softuart_deinit(void)
{
#if (!(PRU1_MODE == PRU_MODE_INVALID))
	pru_disable(PRU_NUM1, &pru_arm_iomap);	
#endif
	pru_disable(PRU_NUM0, &pru_arm_iomap);	

	return PRU_SUART_SUCCESS;
}

/*
 * suart Instance open routine
 */
short pru_softuart_open(suart_handle hSuart)
{
	short status = PRU_SUART_SUCCESS;

	switch (hSuart->uartNum) {
		/* ************ PRU 0 ************** */
	case PRU_SUART_UART1:
		if (gUartStatuTable[PRU_SUART_UART1 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {
			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART1_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART1_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART1_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART1 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}

		break;

	case PRU_SUART_UART2:
		if (gUartStatuTable[PRU_SUART_UART2 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {

			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART2_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART2_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART2_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART2 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}

		break;

	case PRU_SUART_UART3:
		if (gUartStatuTable[PRU_SUART_UART3 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {

			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART3_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART3_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART3_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART3 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}

		break;

	case PRU_SUART_UART4:
		if (gUartStatuTable[PRU_SUART_UART4 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {

			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART4_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART4_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART4_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART4 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}
		break;

		/* ************ PRU 1 ************** */
	case PRU_SUART_UART5:
		if (gUartStatuTable[PRU_SUART_UART5 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {
			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART5_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART5_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART5_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART5 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}
		break;

	case PRU_SUART_UART6:
		if (gUartStatuTable[PRU_SUART_UART6 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {
			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART6_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART6_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART6_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART6 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}

		break;

	case PRU_SUART_UART7:
		if (gUartStatuTable[PRU_SUART_UART7 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {
			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART7_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART7_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART7_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART7 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}
		break;

	case PRU_SUART_UART8:
		if (gUartStatuTable[PRU_SUART_UART8 - 1] ==
		    ePRU_SUART_UART_IN_USE) {
			status = SUART_UART_IN_USE;
			return status;
		} else {
			hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
			hSuart->uartType = PRU_SUART8_CONFIG_DUPLEX;
			hSuart->uartTxChannel = PRU_SUART8_CONFIG_TX_SER;
			hSuart->uartRxChannel = PRU_SUART8_CONFIG_RX_SER;

			gUartStatuTable[PRU_SUART_UART8 - 1] =
			    ePRU_SUART_UART_IN_USE;
		}
		break;

	default:
		/* return invalid UART */
		status = SUART_INVALID_UART_NUM;
		break;
	}
	return (status);
}
//EXPORT_SYMBOL(pru_softuart_open);

/*
 * suart instance close routine 
 */
short pru_softuart_close(suart_handle hUart)
{
	short status = SUART_SUCCESS;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	} else {
		gUartStatuTable[hUart->uartNum - 1] = ePRU_SUART_UART_FREE;
		/* Reset the Instance to Invalid */
		hUart->uartNum = PRU_SUART_UARTx_INVALID;
		hUart->uartStatus = ePRU_SUART_UART_FREE;
	}
	return (status);
}
//EXPORT_SYMBOL(pru_softuart_close);

/*
 * suart routine for setting relative baud rate 
 */
short pru_softuart_setbaud(suart_handle hUart, unsigned short txClkDivisor,
                           unsigned short rxClkDivisor)
{
	unsigned int offset;
	unsigned int pruOffset;
	short status = SUART_SUCCESS;
	unsigned short chNum;
	unsigned short regval = 0;

	if (hUart == NULL) 
	{
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	/* Set the clock divisor value into the McASP */
	if ((txClkDivisor > 385) || (txClkDivisor == 0))
	{
		return SUART_INVALID_CLKDIVISOR;
	}

	if ((rxClkDivisor > 385) || (rxClkDivisor == 0))
	{
		return SUART_INVALID_CLKDIVISOR;
	}

	
	chNum = hUart->uartNum - 1;
 
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}
	
	if (txClkDivisor != 0) 
	{
		offset =
			pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
			PRU_SUART_CH_CONFIG1_OFFSET;
		pru_ram_read_data(offset, (u8 *) &regval, 2,
		                  &pru_arm_iomap);
		regval &= (~0x3FF);
		regval |= txClkDivisor;
		pru_ram_write_data(offset, (u8 *) &regval, 2,
				   &pru_arm_iomap);
	}

	if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}	
	else	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		chNum++;
	}	
	else
	{
		return PRU_MODE_INVALID;
	}	
	
	regval = 0;
	if (rxClkDivisor != 0) 
	{
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;
		pru_ram_read_data(offset, (u8 *) &regval, 2,
				   &pru_arm_iomap);
		regval &= (~0x3FF);
		regval |= txClkDivisor;
		pru_ram_write_data(offset, (u8 *) &regval, 2,
				   &pru_arm_iomap);
	}
	return status;
}

/*
 * suart routine for setting number of bits per character for a specific uart 
 */
short pru_softuart_setdatabits
    (suart_handle hUart, unsigned short txDataBits, unsigned short rxDataBits)
{
	unsigned int offset;
	unsigned int pruOffset;
	short status = SUART_SUCCESS;
	unsigned short chNum;
	unsigned int  reg_val;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	/*
	 * NOTE:
	 * The supported data bits are 6, 7, 8, 9, 10, 11 and 12 bits per character
	 */

	if ((txDataBits < ePRU_SUART_DATA_BITS6) || (txDataBits > ePRU_SUART_DATA_BITS12)) {
		return PRU_SUART_ERR_PARAMETER_INVALID;
	}

	if ((rxDataBits < ePRU_SUART_DATA_BITS6) || (rxDataBits > ePRU_SUART_DATA_BITS12)) {
		return PRU_SUART_ERR_PARAMETER_INVALID;
	}

	chNum = hUart->uartNum - 1;
	
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}


	if (txDataBits != 0) {
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG2_OFFSET;
		pru_ram_read_data(offset, (u8 *) &reg_val, 1,
					   &pru_arm_iomap);

		reg_val &= ~(0xF);
		reg_val |= txDataBits;
		pru_ram_write_data(offset, (u8 *) &reg_val, 1,
				   &pru_arm_iomap);
	}

	if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}	
	else	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		chNum++;
	}	
	else
	{
		return PRU_MODE_INVALID;
	}

	if (rxDataBits != 0) {
		offset =
			pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
			PRU_SUART_CH_CONFIG2_OFFSET;

		pru_ram_read_data(offset, (u8 *) &reg_val, 1,
		                  &pru_arm_iomap);

		reg_val &= ~(0xF);
		reg_val |= rxDataBits;

		pru_ram_write_data(offset, (u8 *) & rxDataBits, 1,
				   &pru_arm_iomap);
	}

	return (status);
}

/*
 * suart routine to configure specific uart 
 */
short pru_softuart_setconfig(suart_handle hUart, suart_config * configUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	short status = SUART_SUCCESS;
	unsigned short chNum;
	unsigned short regVal = 0;
	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	/*
	 * NOTE:
	 * Dependent baud rate for the given UART, the value MUST BE LESS THAN OR 
	 * EQUAL TO 64, preScalarValue <= 64
	 */
	/* Validate the value of relative buad rate */
	if ((configUart->txClkDivisor > 384) || (configUart->rxClkDivisor > 384)) {
		return SUART_INVALID_CLKDIVISOR;
	}
	/* Validate the bits per character */
	if ((configUart->txBitsPerChar < 8) || (configUart->txBitsPerChar > 14)) {
		return PRU_SUART_ERR_PARAMETER_INVALID;
	}

	if ((configUart->rxBitsPerChar < 8) || (configUart->rxBitsPerChar > 14)) {
		return PRU_SUART_ERR_PARAMETER_INVALID;
	}

	chNum = hUart->uartNum - 1;
	
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
	
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;
		
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	/* Configuring the Transmit part of the given UART */
	if (configUart->TXSerializer != PRU_SUART_SERIALIZER_NONE) {
		/* Serializer has been as TX in mcasp config, by writing 1 in bits corresponding to tx serializer 
		   in PFUNC regsiter i.e. already set to GPIO mode PRU code will set then back to MCASP mode once
		   TX request for that serializer is posted. It is required because at this point Mcasp is accessed
		   by both PRU and DSP have lower priority for Mcasp in comparison to PRU and DPS keeps on looping 
		   there only. 
		 */

		/*
		  suart_mcasp_tx_serialzier_set (configUart->TXSerializer, &pru_arm_iomap);
		*/

		/* Configuring TX serializer  */
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CTRL_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regVal, 2,
				  &pru_arm_iomap);

		regVal = (configUart->TXSerializer <<
			      PRU_SUART_CH_CTRL_SR_SHIFT);

		pru_ram_write_data(offset, (u8 *) & regVal, 2,
				   &pru_arm_iomap);
		/* Configuring the Transmit part of the given UART */
		/* Configuring TX prescalar value */
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regVal, 2,
				  &pru_arm_iomap);
		regVal =
		    regVal | (configUart->txClkDivisor <<
			      PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);
		pru_ram_write_data(offset, (u8 *) & regVal, 2,
				   &pru_arm_iomap);
		/* Configuring TX bits per character value */
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG2_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regVal, 2,
				  &pru_arm_iomap);
		regVal =
		    regVal | (configUart->txBitsPerChar <<
			      PRU_SUART_CH_CONFIG2_BITPERCHAR_SHIFT);
		pru_ram_write_data(offset, (u8 *) & regVal, 2,
				   &pru_arm_iomap);
	}

	if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
		pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}	
	else	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		chNum++;
	}	
	else
	{
		return PRU_MODE_INVALID;
	}

	/* Configuring the Transmit part of the given UART */
	if (configUart->RXSerializer != PRU_SUART_SERIALIZER_NONE) {
		/* Configuring RX serializer  */
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CTRL_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regVal, 2,
				  &pru_arm_iomap);

		regVal =  (configUart->RXSerializer <<
				 PRU_SUART_CH_CTRL_SR_SHIFT);
		pru_ram_write_data(offset, (u8 *) & regVal, 2,
				   &pru_arm_iomap);

		/* Configuring RX prescalar value and Oversampling */
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regVal, 2,
				  &pru_arm_iomap);
		regVal =
		    regVal | (configUart->rxClkDivisor <<
			      PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT) |
		    (configUart->Oversampling <<
		     PRU_SUART_CH_CONFIG1_OVS_SHIFT);
		pru_ram_write_data(offset, (u8 *) & regVal, 2,
				   &pru_arm_iomap);

		/* Configuring RX bits per character value */
		offset =
		    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG2_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regVal, 2,
				  &pru_arm_iomap);
		regVal =
		    regVal | (configUart->rxBitsPerChar <<
			      PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);
		pru_ram_write_data(offset, (u8 *) & regVal, 2,
				   &pru_arm_iomap);
	}
	return (status);
}

/*
 * suart routine for getting the number of bytes transfered
 */
short pru_softuart_getTxDataLen(suart_handle hUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short chNum;
	unsigned short u16ReadValue = 0;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1;
	
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;
	
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}	

	/* Transmit channel number is (UartNum * 2) - 2  */

	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG2_OFFSET;
	pru_ram_read_data(offset, (u8 *) & u16ReadValue, 2, &pru_arm_iomap);

	u16ReadValue = ((u16ReadValue & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
			PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);
	return (u16ReadValue);
}

/*
 * suart routine for getting the number of bytes received
 */
short pru_softuart_getRxDataLen(suart_handle hUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short chNum;
	unsigned short u16ReadValue = 0;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1;
	
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;
	
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
		
		chNum++;
	}	
	else if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}
	
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG2_OFFSET;
	pru_ram_read_data(offset, (u8 *) & u16ReadValue, 2, &pru_arm_iomap);

	u16ReadValue = ((u16ReadValue & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
			PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);

	return (u16ReadValue);
}

/*
 * suart routine to get the configuration information from a specific uart 
 */
short pru_softuart_getconfig(suart_handle hUart, suart_config * configUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short chNum;
	unsigned short regVal = 0;
	short status = SUART_SUCCESS;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	/*
	 * NOTE:
	 * Dependent baud rate for the given UART, the value MUST BE LESS THAN OR 
	 * EQUAL TO 64, preScalarValue <= 64
	 */

	chNum = hUart->uartNum - 1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

	
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	/* Configuring the Transmit part of the given UART */
	/* Configuring TX serializer  */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CTRL_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	configUart->TXSerializer =
	    ((regVal & PRU_SUART_CH_CTRL_SR_MASK) >>
	     PRU_SUART_CH_CTRL_SR_SHIFT);

	/* Configuring TX prescalar value */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG1_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	configUart->txClkDivisor =
	    ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
	     PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);

	/* Configuring TX bits per character value */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG2_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	configUart->txBitsPerChar =
	    ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
	     PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);
		 
	if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}	
	else	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		chNum++;
	}	
	else
	{
		return PRU_MODE_INVALID;
	}

	/* Configuring the Transmit part of the given UART */
	/* Configuring RX serializer  */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CTRL_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	configUart->RXSerializer =
	    ((regVal & PRU_SUART_CH_CTRL_SR_MASK) >>
	     PRU_SUART_CH_CTRL_SR_SHIFT);

	/* Configuring RX prescalar value and Oversampling */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG1_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	configUart->rxClkDivisor =
	    ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
	     PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);
	configUart->Oversampling =
	    ((regVal & PRU_SUART_CH_CONFIG1_OVS_MASK) >>
	     PRU_SUART_CH_CONFIG1_OVS_SHIFT);

	/* Configuring RX bits per character value */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG2_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	configUart->rxBitsPerChar =
	    ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
	     PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);

	return (status);
}

/*
 * suart data transmit routine 
 */
short pru_softuart_write
    (suart_handle hUart, unsigned int *ptTxDataBuf, unsigned short dataLen) {
	unsigned int offset = 0;
	unsigned int pruOffset;
	short status = SUART_SUCCESS;
	unsigned short chNum;
	unsigned short regVal = 0;

	unsigned short pru_num;
	
	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1;
	
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
	
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
			pru_num = hUart->uartNum;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
			pru_num = hUart->uartNum;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
		pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		pru_num = 0;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
		pru_num = 1;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	/* Writing data length to SUART channel register */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG2_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	regVal &= ~PRU_SUART_CH_CONFIG2_DATALEN_MASK;
	regVal = regVal | (dataLen << PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);
	pru_ram_write_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);

	/* Writing the data pointer to channel TX data pointer */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_TXRXDATA_OFFSET;
	pru_ram_write_data(offset, (u8 *) ptTxDataBuf, 4, &pru_arm_iomap);

	/* Service Request to PRU */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CTRL_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);

	regVal &= ~(PRU_SUART_CH_CTRL_MODE_MASK |PRU_SUART_CH_CTRL_SREQ_MASK);

	regVal |= (PRU_SUART_CH_CTRL_TX_MODE << PRU_SUART_CH_CTRL_MODE_SHIFT) | 
			 (PRU_SUART_CH_CTRL_SREQ    <<    PRU_SUART_CH_CTRL_SREQ_SHIFT);

	pru_ram_write_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);

	/* generate ARM->PRU event */
	suart_arm_to_pru_intr(pru_num);

	return (status);
}

/*
 * suart data receive routine 
 */
short pru_softuart_read
    (suart_handle hUart, unsigned int *ptDataBuf, unsigned short dataLen) {
	unsigned int offset = 0;
	unsigned int pruOffset;
	short status = SUART_SUCCESS;
	unsigned short chNum;
	unsigned short regVal = 0;
	unsigned short pru_num;
	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1;

	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
			pru_num = hUart->uartNum;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
			pru_num = hUart->uartNum;
		}
		chNum++;	
	}	
	else if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
			pru_num = 0;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
		pru_num = 1;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	/* Writing data length to SUART channel register */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CONFIG2_OFFSET;
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	regVal &= ~PRU_SUART_CH_CONFIG2_DATALEN_MASK;
	regVal = regVal | (dataLen << PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);
	pru_ram_write_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);

	/* Writing the data pointer to channel RX data pointer */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_TXRXDATA_OFFSET;
	pru_ram_write_data(offset, (u8 *) ptDataBuf, 4, &pru_arm_iomap);

	/* Service Request to PRU */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_CTRL_OFFSET;

	
	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);

	regVal &= ~(PRU_SUART_CH_CTRL_MODE_MASK |PRU_SUART_CH_CTRL_SREQ_MASK);

	regVal |=  ( PRU_SUART_CH_CTRL_RX_MODE << PRU_SUART_CH_CTRL_MODE_SHIFT) | 
				(PRU_SUART_CH_CTRL_SREQ << PRU_SUART_CH_CTRL_SREQ_SHIFT);
				
	pru_ram_write_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	
	/* enable the timeout interrupt */
	suart_intr_setmask (hUart->uartNum, PRU_RX_INTR, CHN_TXRX_IE_MASK_TIMEOUT);

	/* generate ARM->PRU event */
	suart_arm_to_pru_intr(pru_num);

	return (status);
}

/* 
 * suart routine to read the data from the RX FIFO
 */
short pru_softuart_read_data (
			suart_handle hUart, 
			u8 * pDataBuffer, 
			s32 s32MaxLen, 
			u32 * pu32DataRead)
{
	short retVal = PRU_SUART_SUCCESS;
	u8 * pu8SrcAddr = NULL;
	u32  u32DataRead = 0;
	u32	u32DataLen = 0;
	u32	u32CharLen = 0;
	unsigned int offset = 0;
	unsigned int pruOffset;
	unsigned short chNum;
	unsigned short u16Status = 0;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum  - 1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
		chNum++;
	}	
	else if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}
	
	/* Get the data pointer from channel RX data pointer */
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_TXRXDATA_OFFSET;
	pru_ram_read_data(offset, (u8 *) &pu8SrcAddr, 4, &pru_arm_iomap);

	/* Reading data length from SUART channel register */
	offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
			 PRU_SUART_CH_CONFIG2_OFFSET;
	pru_ram_read_data(offset, (u8 *) & u32DataLen, 2, &pru_arm_iomap);
	
	/* read the character length */
	u32CharLen = u32DataLen & PRU_SUART_CH_CONFIG2_BITPERCHAR_MASK;
	u32CharLen -= 2; /* remove the START & STOP bit */
	
	u32DataLen &= PRU_SUART_CH_CONFIG2_DATALEN_MASK;
	u32DataLen = u32DataLen >> PRU_SUART_CH_CONFIG2_DATALEN_SHIFT;
	u32DataLen ++;	
	
	/* if the character length is greater than 8, then the size doubles */
	if (u32CharLen > 8)
	{
		u32DataLen *= 2;
	}
		
	/* Check if the time-out had occured. If, yes, then we need to find the 
	 * number of bytes read from PRU. Else, we need to read the requested bytes
	 */
	offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
			 PRU_SUART_CH_TXRXSTATUS_OFFSET;
	pru_ram_read_data(offset, (u8 *) & u16Status, 1, &pru_arm_iomap);
	
	if (u16Status & CHN_TXRX_STATUS_TIMEOUT)
	{
		/* determine the number of bytes read into the FIFO */
		offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
				 PRU_SUART_CH_BYTESDONECNTR_OFFSET;
				 
		pru_ram_read_data(offset, (u8 *) & u32DataRead, 1, &pru_arm_iomap);
		
		/* if the character length is greater than 8, then the size doubles */
		if (u32CharLen > 8)
		{
			u32DataRead *= 2;
		}
		
		/* the data corresponding is loaded in second half during the timeout */
		if (u32DataRead > u32DataLen)
		{
			u32DataRead -= u32DataLen;
			pu8SrcAddr +=  u32DataLen; 
		}
		
		pru_softuart_clrRxFifo (hUart);
	}
	else
	{
		u32DataRead = u32DataLen;
		/* Determine the buffer index by reading the FIFO_OddEven flag*/
		if (u16Status & CHN_TXRX_STATUS_CMPLT)
		{
			/* if the bit is set, the data is in the first half of the FIFO else
			 * the data is in the second half
			 */
			pu8SrcAddr += u32DataLen;
		}
	}

	/* we should be copying only max len given by the application */
	if (u32DataRead > s32MaxLen)
	{
		u32DataRead = s32MaxLen;
	}
	
	/* evaluate the virtual address of the FIFO address based on the physical addr */
	pu8SrcAddr = (u8 *) ((u32) pu8SrcAddr - pru_arm_iomap.pFifoBufferPhysBase + 
							(u32) pru_arm_iomap.pFifoBufferVirtBase);
	
	/* Now we have both the data length and the source address. copy */
	for (offset = 0; offset < u32DataRead; offset++)
		{
		* pDataBuffer++ = * pu8SrcAddr++;
		}
	* pu32DataRead = u32DataRead;	
	
	retVal = PRU_SUART_SUCCESS;
	
	return (retVal);
}

/*
 * suart routine to disable the receive functionality. This routine stops the PRU
 * from receiving on selected UART and also disables the McASP serializer corresponding
 * to this UART Rx line.
 */
short pru_softuart_stopReceive (suart_handle hUart)
{
	unsigned short status = SUART_SUCCESS;
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short chNum;
	unsigned short u16Status;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
	
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;	
	
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
		chNum++;
	}	
	else if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
		pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}
	
	/* read the existing value of status flag */
	offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
			 PRU_SUART_CH_TXRXSTATUS_OFFSET;
	pru_ram_read_data(offset, (u8 *) &u16Status, 1, &pru_arm_iomap);
	
	/* we need to clear the busy bit corresponding to this receive channel */
	u16Status &= ~(CHN_TXRX_STATUS_RDY);
	pru_ram_write_data(offset, (u8 *) & u16Status, 1, &pru_arm_iomap);
	
	/* get the serizlizer number being used for this Rx channel */
	offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
			 PRU_SUART_CH_CTRL_OFFSET;
	pru_ram_read_data(offset, (u8 *) &u16Status, 2, &pru_arm_iomap);
	u16Status &= PRU_SUART_CH_CTRL_SR_MASK;
	u16Status = u16Status >> PRU_SUART_CH_CTRL_SR_SHIFT;	
	
	/* we need to de-activate the serializer corresponding to this receive */
	status =  suart_asp_serializer_deactivate (u16Status, &pru_arm_iomap);
	
	return (status);

}

/*
 * suart routine to get the tx status for a specific uart 
 */
short pru_softuart_getTxStatus(suart_handle hUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short status = SUART_SUCCESS;
	unsigned short chNum;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1 ;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

	
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_TXRXSTATUS_OFFSET;
	pru_ram_read_data(offset, (u8 *) & status, 1, &pru_arm_iomap);
	return (status);
}

short pru_softuart_clrTxStatus(suart_handle hUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short status = SUART_SUCCESS;
	unsigned short chNum;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}
	
	chNum = hUart->uartNum - 1 ;
	
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{

		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;
	
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
	}	
	else if (PRU0_MODE == PRU_MODE_TX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_TX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}
	
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_TXRXSTATUS_OFFSET;
	pru_ram_read_data(offset, (u8 *) & status, 1, &pru_arm_iomap);

	status &= ~(0x2);
	pru_ram_write_data(offset, (u8 *) & status, 1, &pru_arm_iomap);
	return (status);
}

/*
 * suart routine to get the rx status for a specific uart 
 */
short pru_softuart_getRxStatus(suart_handle hUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short status = SUART_SUCCESS;
	unsigned short chNum;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1 ;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;
	
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
		chNum++;
	}	
	else if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}
	
	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_TXRXSTATUS_OFFSET;
	pru_ram_read_data(offset, (u8 *) &status, 1, &pru_arm_iomap);
	return (status);
}

short pru_softuart_clrRxFifo(suart_handle hUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short status = SUART_SUCCESS;
	unsigned short chNum;
	unsigned short regVal;
	unsigned short uartNum;
	
	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	uartNum = hUart->uartNum;

	chNum = hUart->uartNum - 1;
	
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
	
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;
		
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
		chNum++;
	}	
	else if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
		pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		uartNum = 0;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
		uartNum = 1;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	/* Reset the number of bytes read into the FIFO */
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
                         PRU_SUART_CH_BYTESDONECNTR_OFFSET;

        pru_ram_read_data(offset, (u8 *) & regVal, 1, &pru_arm_iomap);
	regVal &= 0x00;
	
        pru_ram_write_data(offset, (u8 *) & regVal, 1, &pru_arm_iomap);

	/* Service Request to PRU */
	offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
			 PRU_SUART_CH_CTRL_OFFSET;

	pru_ram_read_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);

	regVal &= ~(PRU_SUART_CH_CTRL_MODE_MASK |PRU_SUART_CH_CTRL_SREQ_MASK);

	regVal |=  ( PRU_SUART_CH_CTRL_RX_MODE << PRU_SUART_CH_CTRL_MODE_SHIFT) | 
				(PRU_SUART_CH_CTRL_SREQ << PRU_SUART_CH_CTRL_SREQ_SHIFT);

	pru_ram_write_data(offset, (u8 *) & regVal, 2, &pru_arm_iomap);
	suart_intr_setmask (hUart->uartNum, PRU_RX_INTR, CHN_TXRX_IE_MASK_TIMEOUT);
	
	/* generate ARM->PRU event */
	suart_arm_to_pru_intr(uartNum);

	return (status);
}


short pru_softuart_clrRxStatus(suart_handle hUart)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short status = SUART_SUCCESS;
	unsigned short chNum;

	if (hUart == NULL) {
		return (PRU_SUART_ERR_HANDLE_INVALID);
	}

	chNum = hUart->uartNum - 1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
	
		/* channel starts from 0 and uart instance starts from 1 */
		chNum = (hUart->uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;
		
		if (hUart->uartNum <= 4) 
		{
			/* PRU0 */
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		} else {
			/* PRU1 */
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chNum -= 8;
		}
		chNum++;
	}	
	else if (PRU0_MODE == PRU_MODE_RX_ONLY )
	{
			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
	}		
	else if (PRU1_MODE == PRU_MODE_RX_ONLY)
	{
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	offset =
	    pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
	    PRU_SUART_CH_TXRXSTATUS_OFFSET;
	pru_ram_read_data(offset, (u8 *) & status, 1, &pru_arm_iomap);

	status &= ~(0x3C);
	pru_ram_write_data(offset, (u8 *) & status, 1, &pru_arm_iomap);
	return (status);
}

int pru_intr_clr_isrstatus(unsigned short uartNum, unsigned int txrxmode)
{
	unsigned int offset;
	unsigned short txrxFlag = 0;
	unsigned short chnNum;

	chnNum = uartNum - 1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chnNum = (uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if (uartNum <= 4) 
		{
			/* PRU0 */
			offset = PRU_SUART_PRU0_ISR_OFFSET + 1;
		} else {
			/* PRU1 */
			offset = PRU_SUART_PRU1_ISR_OFFSET + 1;
			/* First 8 channel corresponds to PRU0 */
			chnNum -= 8;
		}
		
		if (2 == txrxmode)
			chnNum++;
	}	
	else if (PRU0_MODE == txrxmode)
	{
			offset = PRU_SUART_PRU0_ISR_OFFSET + 1;
	}		
	else if (PRU1_MODE == txrxmode)
	{
		offset = PRU_SUART_PRU1_ISR_OFFSET + 1;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}		
		
	pru_ram_read_data(offset, (u8 *) & txrxFlag, 1, &pru_arm_iomap);
	txrxFlag &= ~(0x2);
	pru_ram_write_data(offset, (u8 *) & txrxFlag, 1, &pru_arm_iomap);

	return 0;
}

short suart_arm_to_pru_intr(unsigned short uartNum)
{
	unsigned int pruNum;

	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
		if ((uartNum > 0) && (uartNum <= 4))
			pruNum = PRU_NUM0;
		else if ((uartNum > 4) && (uartNum <= 8))
			pruNum = PRU_NUM1;
		else
			return SUART_INVALID_UART_NUM;
	}

	if ((PRU0_MODE == PRU_MODE_RX_ONLY) || (PRU1_MODE == PRU_MODE_RX_ONLY) || 
	    (PRU0_MODE == PRU_MODE_TX_ONLY) || (PRU1_MODE == PRU_MODE_TX_ONLY))
		pruNum = uartNum;

	irq_set_irqchip_state(pru_arm_iomap.arm_to_pru_irq[pruNum], IRQCHIP_STATE_PENDING, true);

	return 0;
}

int suart_intr_setmask(unsigned short uartNum,
		       unsigned int txrxmode, unsigned int intrmask)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned int txrxFlag;
	unsigned int regval = 0;
	unsigned int chnNum;

	chnNum = uartNum -1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chnNum = (uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if ((uartNum > 0) && (uartNum <= 4)) {

			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
			offset = PRU_SUART_PRU0_IMR_OFFSET;
		} else if ((uartNum > 4) && (uartNum <= 8)) {
			/* PRU1 */
			offset = PRU_SUART_PRU1_IMR_OFFSET;
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chnNum -= 8;
		} else {
			return SUART_INVALID_UART_NUM;
		}

		if (2 == txrxmode) {	/* rx mode */
			chnNum++;
		}	
	}
	else if (PRU0_MODE == txrxmode)
	{
		pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		offset = PRU_SUART_PRU0_IMR_OFFSET;
	}		
	else if (PRU1_MODE == txrxmode)
	{
		offset = PRU_SUART_PRU1_IMR_OFFSET;
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	
	regval = 1 << chnNum;

	if (CHN_TXRX_IE_MASK_CMPLT == (intrmask & CHN_TXRX_IE_MASK_CMPLT)) 
	{
		pru_ram_read_data(offset, (u8 *) & txrxFlag, 2,
				  &pru_arm_iomap);
		txrxFlag &= ~(regval);
		txrxFlag |= regval;
		pru_ram_write_data(offset, (u8 *) & txrxFlag, 2,
				   &pru_arm_iomap);
	}

	if ((intrmask & SUART_GBL_INTR_ERR_MASK) == SUART_GBL_INTR_ERR_MASK) {
		regval = 0;
		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(SUART_GBL_INTR_ERR_MASK);
		regval |= (SUART_GBL_INTR_ERR_MASK);
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);

	}
	/* Break Indicator Interrupt Masked */
	if ((intrmask & CHN_TXRX_IE_MASK_FE) == CHN_TXRX_IE_MASK_FE) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_TXRX_IE_MASK_FE);
		regval |= CHN_TXRX_IE_MASK_FE;
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}
	/* Framing Error Interrupt Masked */
	if (CHN_TXRX_IE_MASK_BI == (intrmask & CHN_TXRX_IE_MASK_BI)) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;

		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_TXRX_IE_MASK_BI);
		regval |= CHN_TXRX_IE_MASK_BI;
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}
	/* Timeout error Interrupt Masked */
	if (CHN_TXRX_IE_MASK_TIMEOUT == (intrmask & CHN_TXRX_IE_MASK_TIMEOUT)) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;

		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_TXRX_IE_MASK_TIMEOUT);
		regval |= CHN_TXRX_IE_MASK_TIMEOUT;
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}
	/* Overrun error Interrupt Masked */
	if (CHN_RX_IE_MASK_OVRN == (intrmask & CHN_RX_IE_MASK_OVRN)) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;

		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_RX_IE_MASK_OVRN);
		regval |= CHN_RX_IE_MASK_OVRN;
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}
	return 0;
}

int suart_intr_clrmask(unsigned short uartNum,
		       unsigned int txrxmode, unsigned int intrmask)
{
	unsigned int offset;
	unsigned int pruOffset;
	unsigned short txrxFlag;
	unsigned short regval = 0;
	unsigned short chnNum;

	chnNum = uartNum -1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chnNum = (uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if ((uartNum > 0) && (uartNum <= 4)) {

			pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
			offset = PRU_SUART_PRU0_IMR_OFFSET;
		} else if ((uartNum > 4) && (uartNum <= 8)) {
			/* PRU1 */
			offset = PRU_SUART_PRU1_IMR_OFFSET;
			pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chnNum -= 8;
		} else {
			return SUART_INVALID_UART_NUM;
		}

		if (2 == txrxmode) {	/* rx mode */
			chnNum++;
		}	
	}
	else if (PRU0_MODE == txrxmode) 
	{
		pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
		offset = PRU_SUART_PRU0_IMR_OFFSET;
	}		
	else if (PRU1_MODE == txrxmode)
	{
		offset = PRU_SUART_PRU1_IMR_OFFSET;
		pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}
	
	regval = 1 << chnNum;

	if (CHN_TXRX_IE_MASK_CMPLT == (intrmask & CHN_TXRX_IE_MASK_CMPLT)) {
		pru_ram_read_data(offset, (u8 *) & txrxFlag, 2,
				  &pru_arm_iomap);
		txrxFlag &= ~(regval);
		pru_ram_write_data(offset, (u8 *) & txrxFlag, 2,
				   &pru_arm_iomap);
	}

	if ((intrmask & SUART_GBL_INTR_ERR_MASK) == SUART_GBL_INTR_ERR_MASK) {
		regval = 0;
		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(SUART_GBL_INTR_ERR_MASK);
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);

	}
	/* Break Indicator Interrupt Masked */
	if ((intrmask & CHN_TXRX_IE_MASK_FE) == CHN_TXRX_IE_MASK_FE) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;
		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_TXRX_IE_MASK_FE);
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}
	/* Framing Error Interrupt Masked */
	if (CHN_TXRX_IE_MASK_BI == (intrmask & CHN_TXRX_IE_MASK_BI)) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;

		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_TXRX_IE_MASK_BI);
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}
	
	/* Timeout error Interrupt Masked */
	if (CHN_TXRX_IE_MASK_TIMEOUT == (intrmask & CHN_TXRX_IE_MASK_TIMEOUT)) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;

		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_TXRX_IE_MASK_TIMEOUT);
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}	
	/* Overrun error Interrupt Masked */
	if (CHN_RX_IE_MASK_OVRN == (intrmask & CHN_RX_IE_MASK_OVRN)) {
		regval = 0;
		offset =
		    pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
		    PRU_SUART_CH_CONFIG1_OFFSET;

		pru_ram_read_data(offset, (u8 *) & regval, 2,
				  &pru_arm_iomap);
		regval &= ~(CHN_RX_IE_MASK_OVRN);
		pru_ram_write_data(offset, (u8 *) & regval, 2,
				   &pru_arm_iomap);
	}
	return 0;
}

int suart_intr_getmask(unsigned short uartNum,
		       unsigned int txrxmode, unsigned int intrmask)
{
	unsigned short chnNum;
	unsigned int offset;
	unsigned short txrxFlag;
	unsigned short regval = 1;

	chnNum = uartNum -1;
	if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
	{
		/* channel starts from 0 and uart instance starts from 1 */
		chnNum = (uartNum * SUART_NUM_OF_CHANNELS_PER_SUART) - 2;

		if ((uartNum > 0) && (uartNum <= 4)) {

			offset = PRU_SUART_PRU0_IMR_OFFSET;
		} else if ((uartNum > 4) && (uartNum <= 8)) {
			/* PRU1 */
			offset = PRU_SUART_PRU1_IMR_OFFSET;
			/* First 8 channel corresponds to PRU0 */
			chnNum -= 8;
		} else {
			return SUART_INVALID_UART_NUM;
		}

		if (2 == txrxmode) {	/* rx mode */
			chnNum++;
		}	
	}
	else if (PRU0_MODE == txrxmode) 
	{
		offset = PRU_SUART_PRU0_IMR_OFFSET;
	}		
	else if (PRU1_MODE == txrxmode) 
	{
		offset = PRU_SUART_PRU1_IMR_OFFSET;
	}
	else  
	{
		return PRU_MODE_INVALID;
	}

	regval = regval << chnNum;
	
	pru_ram_read_data(offset, (u8 *) & txrxFlag, 2, &pru_arm_iomap);
	txrxFlag &= regval;
	if (0 == intrmask) {
		if (txrxFlag == 0)
			return 1;
	}

	if (1 == intrmask) {
		if (txrxFlag == regval)
			return 1;
	}
	return 0;
}

static int suart_set_pru_id (unsigned int pru_no)
{
	unsigned int offset;
	unsigned short reg_val = 0;

	if (0 == pru_no)
	{	
		offset = PRU_SUART_PRU0_ID_ADDR;
	}
	else if (1 == pru_no)
	{
		offset = PRU_SUART_PRU1_ID_ADDR;	
	}
	else
    	{
		return PRU_SUART_FAILURE;
	}

	pru_ram_read_data(offset, (u8 *) & reg_val, 1, &pru_arm_iomap);
	reg_val &=~SUART_PRU_ID_MASK;
	reg_val = pru_no;
	pru_ram_write_data(offset, (u8 *) & reg_val, 1, &pru_arm_iomap);

	return PRU_SUART_SUCCESS;
}
/* End of file */
