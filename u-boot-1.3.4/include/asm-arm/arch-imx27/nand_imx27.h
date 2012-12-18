/*
 *  Copyright (c) 2008 Armadeus Systems, <nicolas.colombain@armadeus.com>
 *  Copyright (c) 2008 Eric Jarrige <eric.jarrige@armadeus.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */


#ifndef __IMX27_NAND_H
#define __IMX27_NAND_H

#include <asm/arch/imx-regs.h>

 /* Macros to get byte and bit positions of ECC
 */
#define COLPOS(x)			((x) >> 4)
#define BITPOS(x)			((x)& 0xf)

/**
 * Memory areas of the NFC
 */
#define IMX_NFC_BASE                    (0xD8000000)
#define IMX_NFC_MAIN_AREA0              (0xD8000000)          
#define IMX_NFC_MAIN_AREA1              (0xD8000200)  
#define IMX_NFC_SPARE_AREA0             (0xD8000800)          
#define IMX_NFC_REGS                    (0xD8000E00)

/*
 * NFC registers address offest
 */
#define NFC_OFFSET_BUFSIZE             (0x00) /* Internal SRAM Size */
#define NFC_OFFSET_BLCK_ADD_LOCK       (0x02) /* NAND Flash Block Address for Lock Check */
#define NFC_OFFSET_BUF_ADDR            (0x04) /* Buffer Number for Page Data Transfer To/
                                                 From Flash Memory */
#define NFC_OFFSET_FLASH_ADDR          (0x06) /* NAND Flash Address */
#define NFC_OFFSET_FLASH_CMD           (0x08) /* NAND Flash Command */
#define NFC_OFFSET_CONFIG              (0x0A) /* NFC Internal Buffer Lock Control */
#define NFC_OFFSET_ECC_STATUS_RESULT   (0x0C) /* Controller Status/Result of Flash Operation */
#define NFC_OFFSET_ECC_RSLT_MAIN_AREA  (0x0E) /* ECC Error Position of Main Area Data Error */
#define NFC_OFFSET_ECC_RSLT_SPARE_AREA (0x10) /* ECC Error Position of Spare Area Data Error */
#define NFC_OFFSET_WRPROT              (0x12) /* Nand Flash Write Protection */
#define NFC_OFFSET_UNLOCKSTART_BLKADDR (0x14) /* Start Address for Write Protection Unlock */
#define NFC_OFFSET_UNLOCKEND_BLKADDR   (0x16) /* End Address for Write Protection Unlock */
#define NFC_OFFSET_WRPR_STAT           (0x18) /* Current Nand Flash Write Protection Status */
#define NFC_OFFSET_CONFIG1             (0x1A) /* Nand Flash Operation Configuration 1 */
#define NFC_OFFSET_CONFIG2             (0x1C) /* Nand Flash Operation Configuration 2 */

/*
 * NFC registers
 */
	/* Internal SRAM Size */
#define NFC_BUFSIZE             __REG16(IMX_NFC_REGS + NFC_OFFSET_BUFSIZE)
	/* NAND Flash Block Address for Lock Check */
#define NFC_BLCK_ADD_LOCK       __REG16(IMX_NFC_REGS + NFC_OFFSET_BLCK_ADD_LOCK)
	/* Buffer Number for Page Data Transfer To From Flash Memory */
#define NFC_BUF_ADDR            __REG16(IMX_NFC_REGS + NFC_OFFSET_BUF_ADDR)
	/* NAND Flash Address */
#define NFC_FLASH_ADDR          __REG16(IMX_NFC_REGS + NFC_OFFSET_FLASH_ADDR)
	/* NAND Flash Command */
#define NFC_FLASH_CMD           __REG16(IMX_NFC_REGS + NFC_OFFSET_FLASH_CMD)
	/* NFC Internal Buffer Lock Control */
#define NFC_CONFIG              __REG16(IMX_NFC_REGS + NFC_OFFSET_CONFIG)
	/* Controller Status/Result of Flash Operation */
#define NFC_ECC_STATUS_RESULT   __REG16(IMX_NFC_REGS + NFC_OFFSET_ECC_STATUS_RESULT)
	/* ECC Error Position of Main Area Data Error */
#define NFC_ECC_RSLT_MAIN_AREA  __REG16(IMX_NFC_REGS + NFC_OFFSET_ECC_RSLT_MAIN_AREA)
	/* ECC Error Position of Spare Area Data Error */
#define NFC_ECC_RSLT_SPARE_AREA __REG16(IMX_NFC_REGS + NFC_OFFSET_ECC_RSLT_SPARE_AREA)
	/* Nand Flash Write Protection */
#define NFC_WRPROT              __REG16(IMX_NFC_REGS + NFC_OFFSET_WRPROT)
	/* Start Address for Write Protection Unlock */
#define NFC_UNLOCKSTART_BLKADDR __REG16(IMX_NFC_REGS + NFC_OFFSET_UNLOCKSTART_BLKADDR)
	/* End Address for Write Protection Unlock */
#define NFC_UNLOCKEND_BLKADDR   __REG16(IMX_NFC_REGS + NFC_OFFSET_UNLOCKEND_BLKADDR)
	/* Current Nand Flash Write Protection Status */
#define NFC_WRPR_STAT           __REG16(IMX_NFC_REGS + NFC_OFFSET_WRPR_STAT)
	/* Nand Flash Operation Configuration 1 */
#define NFC_CONFIG1             __REG16(IMX_NFC_REGS + NFC_OFFSET_CONFIG1)
	/* Nand Flash Operation Configuration 2 */
#define NFC_CONFIG2             __REG16(IMX_NFC_REGS + NFC_OFFSET_CONFIG2)

/* NFC_ECC_STATUS_RESULT Status Register Bit Fields */
#define NFC_ECC_STAT_ERM_SHFT 	(2)		    /* ERM shift */
#define NFC_ECC_STAT_ERS_MASK 	(0x03)		/* ERS mask  */
#define NFC_ECC_STAT_ERROR1 	(1<<0)		/* correctable error */
#define NFC_ECC_STAT_ERROR2 	(1<<1)		/* non correctable error */

/* NFC_CONFIG Control Register Bit Fields */
#define NFC_CONFIG_UNLOCKED     (1<<1)		/* unlocked */
#define NFC_CONFIG_LOCKED       (1<<0)		/* locked */
/* NFC_WRPROT Control Register Bit Fields */
#define NFC_WRPROT_UNLOCKBLK    (4<<0)		/* unlock block according to given address range */
#define NFC_WRPROT_LOCKALL 	    (2<<0)		/* lock all */
#define NFC_WRPROT_LOCKTIGHT    (1<<0)		/* lock-tight locked blocks */
/* NFC_WRPR_STAT Status Register Bit Fields */
#define NFC_WRPR_US 	        (1<<2)		/* Unlocked status	*/
#define NFC_WRPR_LS 	        (1<<1)		/* Locked status */
#define NFC_WRPR_LTS 	        (1<<0)		/* Lock-tight Status */
/* NFC_CONFIG1 Control Register Bit Fields */
#define NFC_CONFIG1_CEn 	    (1<<7)		/* Flash force CE */
#define NFC_CONFIG1_RST 	    (1<<6)		/* Reset */
#define NFC_CONFIG1_BIG 	    (1<<5)		/* Big Endian Mode */
#define NFC_CONFIG1_INT_MSK     (1<<4)		/* Mask Interrupt Bit	*/
#define NFC_CONFIG1_ECC_EN 	    (1<<3)		/* ECC operation enable */
#define NFC_CONFIG1_SP_EN 	    (1<<2)		/* Flash spare enable */
/* NFC_CONFIG2 Control Register Bit Fields */
#define NFC_CONFIG2_INT	        (1<<15)		/* Interrupt */
#define NFC_CONFIG2_FDO_STATUS	(4<<3)		/* Flash status output */
#define NFC_CONFIG2_FDO_ID	    (2<<3)		/* Flash ID output */
#define NFC_CONFIG2_FDO_PAGE    (1<<3)		/* Flash data output */
#define NFC_CONFIG2_FDI         (1<<2)		/* Flash data input	*/
#define NFC_CONFIG2_FADD 	    (1<<1)		/* Flash address input */
#define NFC_CONFIG2_FCMD 	    (1<<0)		/* Flash command input */
#endif	/* __IMX27_NAND_H */

#define NAND_CMD_LOCK		0x2a
#define NAND_CMD_LOCK_TIGHT	0x2c
#define NAND_CMD_UNLOCK1	0x23
#define NAND_CMD_UNLOCK2	0x24
#define NAND_CMD_LOCK_STATUS	0x7a
/**
 * @file
 * @brief Definitions for the NFC driver (i.MX27)
 */
