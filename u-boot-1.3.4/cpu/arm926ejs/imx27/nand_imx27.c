/*
 * (C) Copyright 2008 (nc) Armadeus Systems
 *  Copyright (c) 2008 Eric Jarrige <eric.jarrige@armadeus.org>
 *
 * Based on the Linux code of :
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
 */

#include <common.h>
#if defined(CONFIG_CMD_NAND) && !defined(CFG_NAND_LEGACY)

#include <asm/errno.h>
#include <nand.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/nand_imx27.h>

#ifdef CFG_NFC_DEBUG
# define NFC_DEBUG1(fmt, args...) printf(fmt, ##args)
#else
# define NFC_DEBUG1(fmt, args...)
#endif


static u8 g_bSpareOnly;
static u8 g_bStatusRequest;
static u16 g_colAddr;


#define TROP_US_DELAY   10000

#define MIN(x, y)		((x < y) ? x : y)

/*
 * OOB placement block for use with hardware ecc generation
 */
static struct nand_oobinfo nand_hw_eccoob_16 = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 24,
	.eccpos = {6, 7, 8, 9, 10, 11, 22, 23, 24, 25, 26, 27, 38, 39, 40, 41, 42, 43, 54, 55, 56, 57, 58, 59},
	.oobfree = {{12, 10}, {28, 10}, {44, 10}, {60, 4}}
};

static struct nand_oobinfo nand_hw_eccoob_8 = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 24,
	.eccpos = {6, 7, 8, 9, 10, 5, 22, 23, 24, 25, 26, 21, 38, 39, 40, 41, 42, 37, 54, 55, 56, 57, 58, 53},
	.oobfree = {{11, 10}, {27, 10}, {43, 10}, {59, 5}}
};

static uint8_t bbt_pattern[] = {'B', 'b', 't', '0' };
static uint8_t mirror_pattern[] = {'1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	12, 
	.len = 4,
	.veroffs = 4, 
	.maxblocks = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	12, 
	.len = 4,
	.veroffs = 4, 
	.maxblocks = 4,
	.pattern = mirror_pattern
};

static u_char nand_verify_buf[2048];

/*
 * not required for iMX27 NFC
 */
static void nfc_hwcontrol(struct mtd_info *mtdinfo, int cmd)
{
	return;
}

static int nfc_dev_ready(struct mtd_info *mtd)
{
	/* 
	 * NFC handles R/B internally.Therefore,this function
	 * always returns status as ready.
	 */
	return 1;
}

static void wait_op_done(int maxRetries, u16 param)
{
    int hsloop = 1000;
	while (hsloop-- > 0) {
		if (NFC_CONFIG2 & NFC_CONFIG2_INT) {
			NFC_CONFIG2 &= ~NFC_CONFIG2_INT;
			return;
		}
	}
	while (maxRetries-- > 0) {
		if (NFC_CONFIG2 & NFC_CONFIG2_INT) {
			NFC_CONFIG2 &= ~NFC_CONFIG2_INT;
			break;
		}
		udelay(1);
	}
	if (maxRetries <= 0)
		printf("wait_op_done (%x): INT not set\n", NFC_FLASH_CMD);
}


/*!
 * This function issues the specified command to the NAND device and
 * waits for completion.
 *
 * @param       cmd     command for NAND Flash
 */
static void send_cmd(u16 cmd)
{
	NFC_DEBUG1("send_cmd(0x%x)\n", cmd);
	NFC_FLASH_CMD = (u16) cmd;

	NFC_CONFIG2 = NFC_CONFIG2_FCMD;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, cmd);
}

/*!
 * This function sends an address (or partial address) to the
 * NAND device.  The address is used to select the source/destination for
 * a NAND command.
 *
 * @param       addr    address to be written to NFC.
  */
static void send_addr(u16 addr)
{
	NFC_DEBUG1("send_addr(0x%x)\n", addr);

	NFC_FLASH_ADDR = addr;
	NFC_CONFIG2 = NFC_CONFIG2_FADD;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, addr);
}

/*!
 * This function requests the NANDFC to initate the transfer
 * of data currently in the NANDFC RAM buffer to the NAND device.
 *
 * @param	buf_id	      Specify Internal RAM Buffer number (0-3)	
 * @param       bSpareOnly    set 1 if only the spare area is transferred
 */
static void send_prog_page(u8 buf_id, u8 bSpareOnly)
{
	NFC_DEBUG1("send_prog_page (%d)\n", bSpareOnly);

	/* NANDFC buffer 0 is used for page read/write */

	NFC_BUF_ADDR = buf_id;

	/* Configure spare or page+spare access */
	if (!(FMCR & NF_FMS)) {
		if (bSpareOnly) {
			NFC_CONFIG1 |= NFC_CONFIG1_SP_EN;
		} else {
			NFC_CONFIG1 &= ~(NFC_CONFIG1_SP_EN);
		}
	}
	NFC_CONFIG2 = NFC_CONFIG2_FDI;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, bSpareOnly);
}

#ifdef NFC_NAND_SOFT_ECC_CORRECTION
/*!
 * This function will correct the single bit ECC error
 * iMX21 errata: If there are single bit errors in two consecutive page reads
 * then the error is not  corrected by the NFC for the second page.
 * Correct single bit error in driver
 *
 * @param  buf_id	Specify Internal RAM Buffer number (0-3)	
 * @param  eccpos 	Ecc byte and bit position
 * @param  bSpareOnly  	set to 1 if only spare area needs correction
 */

static void nfc_correct_error(u8 buf_id, u16 eccpos, u8 bSpareOnly)
{
	u16 col;
	u8 pos;
	volatile u16 *buf;

	/* Get col & bit position of error
	   these macros works for both 8 & 16 bits */
	col = COLPOS(eccpos);	/* Get half-word position */
	pos = BITPOS(eccpos);	/* Get bit position */

	NFC_DEBUG1("nfc_correct_error (col=%d pos=%d), eccpos=%d\n", col, pos, eccpos);

	/* Set the pointer for main / spare area */
	if (!bSpareOnly) {
		buf = (volatile u16 *)((ulong)IMX_NFC_MAIN_AREA0 + col + (512 * buf_id));
	} else {
		buf = (volatile u16 *)((ulong)IMX_NFC_SPARE_AREA0 + col + (16 * buf_id));
	}

	/* Fix the data */
	*buf ^= (1 << pos);
}
#endif


/*!
 * This function will maintains state of single bit Error
 * in Main & spare  area
 *
 * @param buf_id	Specify Internal RAM Buffer number (0-3)	
 * @param spare  	set to true if only spare area needs correction
 */
static int nfc_correct_ecc(u8 buf_id, u8 spare)
{
	int lerr = 0;
	u16 value, ecc_status_main, ecc_status_spare;
	/* Read the ECC result */
	ecc_status_main = NFC_ECC_STATUS_RESULT >> NFC_ECC_STAT_ERM_SHFT;
	ecc_status_spare = NFC_ECC_STATUS_RESULT & NFC_ECC_STAT_ERS_MASK;

	if (ecc_status_main){
	        if(ecc_status_main == NFC_ECC_STAT_ERROR1) {
			value = NFC_ECC_RSLT_MAIN_AREA;
		    	/* Correct single bit error in Mainarea
		    	   NFC will not correct the error in
		    	   current page */
			NFC_DEBUG1("nfc_correct_ecc: fix error in main area\n");
#ifdef NFC_NAND_SOFT_ECC_CORRECTION
			nfc_correct_error(buf_id, value, 0);
#endif
	        }
	        else if(ecc_status_main == NFC_ECC_STAT_ERROR2) {
			/* 2 bit error in Mainarea !! */
			NFC_DEBUG1("nfc_correct_ecc: 2bit error in main area !!\n");
			lerr = 1;
	        }
	}

	if (ecc_status_spare){
	        if(ecc_status_spare == NFC_ECC_STAT_ERROR1) {
	    		value = NFC_ECC_RSLT_SPARE_AREA;
		    	/* Correct single bit error in sparearea
		    	   NFC will not correct the error in
		    	   current page */
			NFC_DEBUG1("nfc_correct_ecc: fix error in spare area\n");
#ifdef NFC_NAND_SOFT_ECC_CORRECTION
		    	nfc_correct_error(buf_id, value, 1);
#endif
	        }
	        else if(ecc_status_spare == NFC_ECC_STAT_ERROR2) {
		    	/* 2 bit error in Mainarea !! */
			NFC_DEBUG1("nfc_correct_ecc: 2bit error in spare area !!\n");
			lerr = 1;
	        }
	}

	return lerr;
}



/*!
 * This function requests the NANDFC to initated the transfer
 * of data from the NAND device into in the NANDFC ram buffer.
 *
 * @param  	buf_id		Specify Internal RAM Buffer number (0-3)	
 * @param   bSpareOnly  set 1 if only the spare area is transferred
 */
static int send_read_page(u8 buf_id, u8 bSpareOnly)
{
	int lerr = 0;
	NFC_DEBUG1("send_read_page (%d)\n", bSpareOnly);
	/* NANDFC buffer 0 is used for page read/write */
	NFC_BUF_ADDR = buf_id;

	/* Configure spare or page+spare access */
	if (!(FMCR & NF_FMS)) {
		if (bSpareOnly) {
			NFC_CONFIG1 |= NFC_CONFIG1_SP_EN;
		} else {
			NFC_CONFIG1 &= ~(NFC_CONFIG1_SP_EN);
		}
	}

	NFC_CONFIG2 = NFC_CONFIG2_FDO_PAGE;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, bSpareOnly);

	lerr = nfc_correct_ecc(buf_id, bSpareOnly);

	return lerr;
}

/*!
 * This function requests the NANDFC to perform a read of the
 * NAND device ID.
 */
static void send_read_id(struct mtd_info *mtd)
{
 	struct nand_chip *this =  mtd->priv;

    	NFC_DEBUG1("send_read_id \n");
	/* NANDFC buffer 0 is used for device ID output */
	NFC_BUF_ADDR = 0x0;

	/* Read ID into main buffer */
	NFC_CONFIG1 &= (~(NFC_CONFIG1_SP_EN));
	NFC_CONFIG2 = NFC_CONFIG2_FDO_ID;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, 0);

	if (this->options & NAND_BUSWIDTH_16) {
		volatile u16 *mainBuf = (volatile u16 *)((ulong) (IMX_NFC_MAIN_AREA0));

		/*
		 * Pack the every-other-byte result for 16-bit ID reads
		 * into every-byte as the generic code expects and various
		 * chips implement.
		 */

		mainBuf[0] = (mainBuf[0] & 0xff) | ((mainBuf[1] & 0xff) << 8);
		mainBuf[1] = (mainBuf[2] & 0xff) | ((mainBuf[3] & 0xff) << 8);
		mainBuf[2] = (mainBuf[4] & 0xff) | ((mainBuf[5] & 0xff) << 8);
	}
}

static void send_read_lock_status(struct mtd_info *mtd, int page_addr)
{
	int block_addr = page_addr>>6;

    	NFC_DEBUG1("get_lock_status %x\n", NFC_WRPR_STAT);

	if ((NFC_WRPR_STAT & NFC_WRPR_US) && !(NFC_WRPR_STAT & NFC_WRPR_LTS) 
		&& ((NFC_UNLOCKEND_BLKADDR >= block_addr)
		&& (NFC_UNLOCKSTART_BLKADDR <= block_addr))) {
		*(volatile u16 *)((ulong) (IMX_NFC_MAIN_AREA0)) = NFC_WRPR_US;
	} else if (NFC_WRPR_STAT & NFC_WRPR_LTS) {
		*(volatile u16 *)((ulong) (IMX_NFC_MAIN_AREA0))
			= NFC_WRPR_STAT;
	} else {
		*(volatile u16 *)((ulong) (IMX_NFC_MAIN_AREA0))
			= NFC_WRPR_STAT & ~NFC_WRPR_US;
	}
	return;
}

/*!
 * This function writes data of length \b len to buffer \b buf. The data to be
 * written on NAND Flash is first copied to RAMbuffer. After the Data Input
 * Operation by the NFC, the data is written to NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be written to NAND Flash
 * @param       len     number of bytes to be written
 */
static void nfc_write_buf(struct mtd_info *mtd,
			       const u_char * buf, int len)
{
	int n;
	int i = 0;
	volatile u32 *p;

	NFC_DEBUG1("write buf nbbytes len %d, gcol %d\n", len, g_colAddr);

	/* Adjust saved column address */
	if (g_colAddr < mtd->oobblock && g_bSpareOnly)
		g_colAddr += mtd->oobblock;

	p = (volatile u32 *)((ulong)(IMX_NFC_MAIN_AREA0) + (g_colAddr & ~3));

	n = mtd->oobblock + mtd->oobsize - g_colAddr;
	n = min(len, n);

	while (0 < n) {
		int colByteShift = (g_colAddr & 3);
		int m;

		/* adjust 32 bit alignement transfer and last bytes */
		if ((colByteShift) || (4 > n))
		{
			u32 mask = 0;
			int nBytesWrite = 4;
			
			if (n<4) {
				mask = 0xffffffff << (n<<3);
				nBytesWrite = n;
			}

			if (colByteShift) {
				mask = ~(~mask << (colByteShift<<3));
				nBytesWrite -= colByteShift;
			}

			*p = (*p & mask) | (*((u32*)&buf[i]) << (colByteShift <<3));
			p++;
			n -= nBytesWrite;
			g_colAddr += nBytesWrite;
			i += nBytesWrite;
		}

		m = n & ~3;
            	for( ; i<m; i+=4 ){
        		*p++ = *((u32*) &buf[i]);
            	}
		n -= m;
		g_colAddr += m;
	}
 }


/*
 * These functions are quite problematic for the NFC. Luckily they are
 * not used in the current nand code, except for nand_command, which
 * we've defined our own anyway. 
 */
static void nfc_write_word(struct mtd_info *mtd, u16 word)
{
	printf("nfc_write_word: WARNING, this function does not work with the iMX27 NFC!\n");
}
static void nfc_write_byte(struct mtd_info *mtd, u_char byte)
{
	printf("nfc_write_byte: WARNING, this function does not work with the iMX27 NFC!\n");
}


/*!
 * This function id is used to read the data buffer from the NAND Flash. To
 * read the data from NAND Flash first the data output cycle is initiated by
 * the NFC, which copies the data to RAMbuffer. This data of length \b len is
 * then copied to buffer \b buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be read from NAND Flash
 * @param       len     number of bytes to be read
 */
static void nfc_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	int n;
	int i = 0;
	volatile u32 *p;

	NFC_DEBUG1("read buf nbbytes len %d, gcol %d\n", len, g_colAddr);

	/* Adjust saved column address */
	if (g_colAddr < mtd->oobblock && g_bSpareOnly)
		g_colAddr += mtd->oobblock;

	p = (volatile u32 *)((ulong)(IMX_NFC_MAIN_AREA0) + (g_colAddr & ~3));

	n = mtd->oobblock + mtd->oobsize - g_colAddr;
	n = min(len, n);

	while (0 < n) {
		int colByteShift = g_colAddr & 3;
		int m;

		/* adjust 32 bit alignement transfer */		
		if (colByteShift || (4 > n))
		{
			u_char data[4];			
			int lj = 0;
			int nBytesRead = (n<4)? n : 4;

			nBytesRead -= colByteShift;


			*((u32*)data) = *p++ >> (colByteShift<<3);
			n -= nBytesRead;
			g_colAddr += nBytesRead;

			while(nBytesRead) {
				buf[i++] = data[lj++];
				nBytesRead--;	
			}
		}

		m = n & ~3;
		for( ; i<m; i+=4 ){
        		*((u32*) &buf[i]) = *p++;
		}
		g_colAddr += m;
		n -= m;
	}
}

/*
 * read a word. Not implemented as not used in NAND code.
 */
static u16 nfc_read_word(struct mtd_info *mtd)
{
	printf("nfc_read_word: UNIMPLEMENTED.\n");
	return 0;
}

static u16 get_dev_status(void)
{
	volatile u16 *mainBuf = (volatile u16 *)((ulong) (IMX_NFC_MAIN_AREA1));
	u32 store;
	u16 ret;
	/* Issue status request to NAND device */

	/* store the main area1 first word, later do recovery */
	store = *((u32 *) mainBuf);
	/*
	 * NANDFC buffer 1 is used for device status to prevent
	 * corruption of read/write buffer on status requests.
	 */
	NFC_BUF_ADDR = 1;

	/* Send the Read status command before reading status */
	send_cmd(NAND_CMD_STATUS);

	/* Read status into main buffer */
	NFC_CONFIG1 &= (~(NFC_CONFIG1_SP_EN));
	NFC_CONFIG2 = NFC_CONFIG2_FDO_STATUS;

	/* Wait for operation to complete */
	wait_op_done(TROP_US_DELAY, 0);

	/* get status, then recovery area 1 data */
	ret = mainBuf[0];
    	NFC_DEBUG1("get_dev_status %x\n", ret);
	*((u32 *) mainBuf) = store;

	/* Status is placed in first word of main buffer */
	return ret;
}


/*!
 * This function reads byte from the NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static u_char nfc_read_byte(struct mtd_info *mtd)
{
	u_char retVal = 0;
	u16 col, rdWord;
	volatile u16 *mainBuf = (volatile u16 *)((ulong) (IMX_NFC_MAIN_AREA0));
	volatile u16 *spareBuf = (volatile u16 *)((ulong) (IMX_NFC_SPARE_AREA0));

	/* Check for status request */
	if (g_bStatusRequest) {
		return (get_dev_status() & 0xFF);
	}

	/* Get column for 16-bit access */
	col = g_colAddr >> 1;

	/* If we are accessing the spare region */
	if (g_bSpareOnly) {
		rdWord = spareBuf[col];
	} else {
		rdWord = mainBuf[col];
	}

	/* Pick upper/lower byte of word from RAM buffer */
	if (g_colAddr & 0x1) {
		retVal = (rdWord >> 8) & 0xFF;
	} else {
		retVal = rdWord & 0xFF;
	}
    	NFC_DEBUG1("\treval: %x, rdWord:%x \n",retVal, rdWord);

	/* Update saved column address */
	g_colAddr++;

	return retVal;
}

/* this function is called after Programm and Erase Operations to
 * check for success or failure */
static int nfc_wait(struct mtd_info *mtd, struct nand_chip *this, int state)
{
 	return 0;
}

static int nfc_correct_data(struct mtd_info *mtd, u_char * dat,
				 u_char * read_ecc, u_char * calc_ecc)
{
	/*
	 * 1-Bit errors are automatically corrected in HW.  No need for
	 * additional correction.  2-Bit errors cannot be corrected by
	 * HW ECC, so we need to return failure
	 */
	u16 ecc_status = NFC_ECC_STATUS_RESULT;

	if (((ecc_status & NFC_ECC_STAT_ERS_MASK) == NFC_ECC_STAT_ERROR2) || 
        ((ecc_status >> NFC_ECC_STAT_ERM_SHFT) == NFC_ECC_STAT_ERROR2)) {
        NFC_DEBUG1("nfc_correct_data: 2 bit error!\n");
        /*udelay(100000);*/
		return -1;
	}

	return 0;
}

/*!
 * This function is used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash
 *
 * @param       mtd             MTD structure for the NAND Flash
 * @param       command         command for NAND Flash
 * @param       column          column offset for the page read
 * @param       page_addr       page to be read from NAND Flash
 */
static void nfc_cmdfunc(struct mtd_info *mtd, unsigned int command,
			int column, int page_addr)
{
	int lerr = 0;
	struct nand_chip *this =  mtd->priv;
	u8 is2k_Pagesize;
	NFC_DEBUG1("nfc_cmdfunc (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
	      command, column, page_addr);

	if ((FMCR & NF_FMS) == NF_FMS)
		is2k_Pagesize = 1;

	/*
	 * Reset command state information
	 */
	g_bStatusRequest = 0;

	/*
	 * Command pre-processing step
	 */
	switch (command) {

	case NAND_CMD_STATUS:
		g_colAddr = 0;
		g_bStatusRequest = 1;
		break;

	case NAND_CMD_READ0:
		g_colAddr = column;
		g_bSpareOnly = 0;
		break;

	case NAND_CMD_READOOB:
		g_colAddr = column;
		g_bSpareOnly = 1;
		if (is2k_Pagesize)
			command = NAND_CMD_READ0;	/* only READ0 is valid */
		break;

	case NAND_CMD_SEQIN:
		if (column >= mtd->oobblock) {
			if (is2k_Pagesize) {
				/** 			 
				  * FIXME: before send SEQIN command for write OOB,
				  * We must read one page out. 			 
				  * For K9F1GXX has no READ1 command to set current HW 			 
				  * pointer to spare area, we must write the whole page including OOB together.			 
				  */
				/* call itself to read a page */
				nfc_cmdfunc(mtd, NAND_CMD_READ0, 0,
						 page_addr);
			}
			g_colAddr = column - mtd->oobblock;
			g_bSpareOnly = 1;
			/* Set program pointer to spare region */
			if (!is2k_Pagesize)
				send_cmd(NAND_CMD_READOOB);
		} else {
			g_bSpareOnly = 0;
			g_colAddr = column;  
			/* Set program pointer to page start */
			if (!is2k_Pagesize)
				send_cmd(NAND_CMD_READ0);
		}
		break;

	case  NAND_CMD_PAGEPROG: 
            if( this->eccmode == NAND_ECC_NONE )  {
                /* special case for biterr generation */
    	        NFC_CONFIG1 &= ~(NFC_CONFIG1_ECC_EN);
                NFC_DEBUG1 ("nfc_cmdfunc: page prog without ECC\n");
            }
            else{
    		/* Enable Ecc for page writes */
    		NFC_CONFIG1 |= NFC_CONFIG1_ECC_EN;
            }  
    		send_prog_page(0, g_bSpareOnly);

    		if (is2k_Pagesize) {
    			/* data in 4 areas datas */
    			send_prog_page(1, g_bSpareOnly);
    			send_prog_page(2, g_bSpareOnly);
    			send_prog_page(3, g_bSpareOnly);
    		}
    		break;

   	case NAND_CMD_ERASE1: break;
   	case NAND_CMD_LOCK:
		NFC_UNLOCKSTART_BLKADDR = NFC_UNLOCKEND_BLKADDR = -1;
		NFC_WRPROT = NFC_WRPROT_LOCKALL;
		return;
   	case NAND_CMD_LOCK_TIGHT:
		NFC_UNLOCKSTART_BLKADDR = NFC_UNLOCKEND_BLKADDR = -1;
		NFC_WRPROT = NFC_WRPROT_LOCKTIGHT;
		return;
   	case NAND_CMD_UNLOCK1:
		NFC_UNLOCKSTART_BLKADDR= page_addr>>6;
		return;
   	case NAND_CMD_UNLOCK2:
		NFC_UNLOCKEND_BLKADDR= page_addr>>6;
		NFC_WRPROT = NFC_WRPROT_UNLOCKBLK;
		return;
   	case NAND_CMD_LOCK_STATUS:
		g_colAddr = 0;g_bSpareOnly = 0;
		send_read_lock_status(mtd, page_addr);
		return;
    default: break;
	}


	/*
	 * Write out the command to the device.
	 */

	send_cmd(command);

	/*
	 * Write out column address, if necessary
	 */
	if (column != -1) {
		/*
		 * MXC NANDFC can only perform full page+spare or
		 * spare-only read/write.  When the upper layers
		 * layers perform a read/write buf operation,
		 * we will used the saved column adress to index into
		 * the full page.
		 */
		send_addr(0);
		if (is2k_Pagesize)
			send_addr(0);	/* another col addr cycle for 2k page */
	}

	/*
	 * Write out page address, if necessary
	 */
	if (page_addr != -1) {
		send_addr(page_addr & 0xff);	/* paddr_0 - p_addr_7 */

		if (is2k_Pagesize) {
			send_addr((page_addr >> 8) & 0xFF);
			if (mtd->size >= 0x10000000) {
				send_addr((page_addr >> 16) & 0xff);
			}
		} else {
			/* One more address cycle for higher density devices */
			if (mtd->size >= 0x4000000) {
				send_addr((page_addr >> 8) & 0xff);	/* paddr_8 - paddr_15 */
				send_addr((page_addr >> 16) & 0xff);
			} else
				send_addr((page_addr >> 8) & 0xff);	/* paddr_8 - paddr_15 */
		}
	}

	/*
	 * Command post-processing step
	 */
	switch (command) {

	case NAND_CMD_RESET:
			send_cmd(NAND_CMD_RESET);
		break;

	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
		if (is2k_Pagesize) {
			/* send read confirm command */
			send_cmd(NAND_CMD_READSTART);
			/* read for each AREA */
			lerr  = send_read_page(0, g_bSpareOnly);
			lerr |= send_read_page(1, g_bSpareOnly);
			lerr |= send_read_page(2, g_bSpareOnly);
			lerr |= send_read_page(3, g_bSpareOnly);
		} else {
			lerr  = send_read_page(0, g_bSpareOnly);
		}
		if (lerr)
			NFC_DEBUG1 ("Uncorrectable read error at 0x%08X!!\n",
				page_addr<<((is2k_Pagesize)?11:5));
		break;

	case NAND_CMD_READID:
		send_read_id(mtd);
		break;

	case NAND_CMD_PAGEPROG:
#ifdef NFC_NAND_SOFT_ECC_CORRECTION
		/* Disable Ecc after page writes */
		NFC_CONFIG1 &= ~(NFC_CONFIG1_ECC_EN);
#else
		NFC_CONFIG1 |= NFC_CONFIG1_ECC_EN;
#endif
		break;

	case NAND_CMD_STATUS:
		break;

	case NAND_CMD_ERASE2:
		break;
	default: break;
	}
}

/*!
 * This function is used by upper layer for select and deselect of the NAND
 * chip
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       chip    val indicating select or deselect
 */
static void nfc_select_chip(struct mtd_info *mtd, int chip)
{
#ifdef CONFIG_MTD_NAND_MXC_FORCE_CE
	if (chip > 0) {
		NFC_DEBUG1("ERROR:  Illegal chip select (chip = %d)\n", chip);
		return;
	}

	if (chip == -1) {
		NFC_CONFIG1 &= (~(NFC_CONFIG1_CEn));
		return;
	}

	NFC_CONFIG1 |= NFC_CONFIG1_CEn;
#endif
    NFC_DEBUG1("nfc_select_chip: %d)\n", chip);
	switch (chip) {
	case -1:
		/* Disable the NFC clock */
        PCCR1 &= ~PCCR1_NFC_BAUDEN;
		break;
	case 0:
		/* Enable the NFC clock */
        PCCR1 |= PCCR1_NFC_BAUDEN;
		break;

	default:
		break;
	}
}


static void nfc_enable_hwecc(struct mtd_info *mtd, int mode)
{
	/*
	 * If HW ECC is enabled, we turn it on during init.  There is
	 * no need to enable again here.
	 */
}

static int nfc_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
				  u_char * ecc_code)
{
	/*
	 * Just return success.  HW ECC does not read/write the NFC spare
	 * buffer.  Only the FLASH spare area contains the calcuated ECC.
	 */
	struct nand_chip *this = mtd->priv;
	NFC_DEBUG1("nfc_calculate_ecc: len = %d\n", this->eccbytes);
	memset ((void*)ecc_code, 0xff, this->eccbytes);
				
	return 0;
}

/*!
 * This function is used by the upper layer to verify the data in NAND Flash
 * with the data in the \b buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be verified
 * @param       len     length of the data to be verified
 *
 * @return      -EFAULT if error else 0
 *
 */
static int nfc_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
int lerr = 0;

	NFC_DEBUG1("nfc_verify_buf(col = %d, len = %d)\n", g_colAddr, len);

	nfc_read_buf(mtd, (u_char *)&nand_verify_buf[0], len);

	if (!memcmp(buf, &nand_verify_buf[0], len)) {
		NFC_DEBUG1("verify_buf: the buffer is OK\n");
		lerr = 0;
	} else {
		NFC_DEBUG1("verify_buf: the buffer is wrong\n");
		lerr = -EFAULT;
	}
	return lerr;
}

/*!
 * nfc_block_markbad - mark a block bad
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be verified
 * @param       ofs     offset to the bad block 
 *
 * @return      -EFAULT if error else 0
 *
 * This function is used by the upper layer to mark bad blocks
 * We overload the default function to support NAND SPL capabilities to detect
 * badblocks without scanning the BBT. The OOB is fill with 0
 *
 */
static int nfc_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *this = mtd->priv;
	u_char buf[NAND_MAX_OOBSIZE] = {0, 0};
	size_t	retlen;
	int block, i, ret = 0;
	long pages_per_block;

	/* Get block number */
	block = ((int) ofs) >> this->bbt_erase_shift;
	this->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	/* Do we have a flash based bad block table ? */
	if (this->options & NAND_USE_FLASH_BBT)
		nand_update_bbt (mtd, ofs);

	/* fill oob with 0 to make it detecable by SPL */
	memset((void*)buf,0, mtd->oobsize);
	
	/* Calculate pages in each block */
	pages_per_block = 1 << (this->phys_erase_shift - this->page_shift);

	for(i=0; i< pages_per_block;i++) {
		ret= mtd->write_oob (mtd, ofs , NAND_MAX_OOBSIZE, &retlen, buf); 
		ofs += 1 << this->page_shift;
	}

	return ret;
}

/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific (per include/linux/mtd/nand_new.h):
 * - IO_ADDR_R?: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W?: address to write the 8 I/O lines of the flash device
 * - hwcontrol: hardwarespecific function for accesing control-lines
 * - dev_ready: hardwarespecific function for  accesing device ready/busy line
 * - enable_hwecc?: function to enable (reset)  hardware ecc generator. Must
 *   only be provided if a hardware ECC is available
 * - eccmode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 * Members with a "?" were not set in the merged testing-NAND branch,
 * so they are not set here either.
 */
int board_nand_init(struct nand_chip *nand)
{
	g_colAddr = 0;

	/* 50 us command delay time */
	nand->chip_delay = 5;

	nand->hwcontrol = nfc_hwcontrol;
	nand->waitfunc = nfc_wait;   
	nand->read_byte = nfc_read_byte;
	nand->write_byte = nfc_write_byte;
	nand->read_word = nfc_read_word;
	nand->write_word = nfc_write_word;
	nand->read_buf = nfc_read_buf;
	nand->write_buf = nfc_write_buf;
    	nand->dev_ready =nfc_dev_ready;
    	nand->select_chip = nfc_select_chip;
    	nand->verify_buf = nfc_verify_buf;
	nand->block_markbad = nfc_block_markbad;
	nand->cmdfunc = nfc_cmdfunc;

	/* hardware ECC */
	nand->calculate_ecc = nfc_calculate_ecc; 
	nand->enable_hwecc = nfc_enable_hwecc;  
	nand->correct_data = nfc_correct_data;
	nand->eccsize = 512;
	nand->eccbytes = 3;
	nand->eccmode = NAND_ECC_HW6_512;
#ifdef NFC_NAND_SOFT_ECC_CORRECTION
	/*
	 * Fix iMX21 NFC bug: NFC fails to correct two single-bit errors if
	 * they occur on consecutive pages, on data read accesses.
	 *
	 * NFC automatic ECC calculation is done during write
	 * NFC ECC correction is done during read upon software control 
	 */	
	NFC_CONFIG1 &= ~(NFC_CONFIG1_ECC_EN);
#else
	NFC_CONFIG1 |= NFC_CONFIG1_ECC_EN;
#endif
	NFC_CONFIG1 |= NFC_CONFIG1_INT_MSK; /* disable interrupt */

	/* NAND bus width determines access funtions used by upper layer */
	if (FMCR & NF_16BIT_SEL) {
		nand->options |= NAND_BUSWIDTH_16 | NAND_USE_FLASH_BBT;
		nand->autooob = &nand_hw_eccoob_16;
	} else {
		nand->options |= NAND_USE_FLASH_BBT;
		nand->autooob = &nand_hw_eccoob_8;
	}

	nand->bbt_td = &bbt_main_descr;
	nand->bbt_md = &bbt_mirror_descr;
	/* preset operation */
	/* Unlock the internal RAM Buffer */
	NFC_CONFIG = NFC_CONFIG_UNLOCKED;

	/* Blocks to be unlocked */
	NFC_UNLOCKSTART_BLKADDR = 0x0;
	NFC_UNLOCKEND_BLKADDR = 0xFFFF;

	/* Unlock Block Command for given address range */
	NFC_WRPROT = NFC_WRPROT_UNLOCKBLK;

	send_cmd(NAND_CMD_RESET);

	return 0;
}

#endif
