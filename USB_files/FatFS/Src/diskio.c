/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "flash.h"
#include "string.h"
#include "stdio.h" 
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE* buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	W25Q128_read_data(sector * FF_MAX_SS, buff, count * FF_MAX_SS);

	return RES_OK;

}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write(
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE* buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{

	W25Q128_modify_data(sector * FF_MAX_SS, (uint8_t*)buff, count * FF_MAX_SS);
	return RES_OK;;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void* buff		/* Buffer to send/receive control data */
)
{
	switch (cmd)
	{
	case CTRL_SYNC:
		while (W25Q128_check_if_busy());
		break;
	case GET_SECTOR_COUNT:
		*(WORD*)buff = W25Q128_SECTOR_COUNT;
		break;
	case GET_SECTOR_SIZE:
		*(WORD*)buff = W25Q128_SECTOR_SIZE;
		break;
	case GET_BLOCK_SIZE:
		*(WORD*)buff = 1;
		break;
	case CTRL_TRIM:
		// not used (FF_USE_TRIM == 0)
		break;
	default:
		break;
	}
	return RES_OK;

	return RES_PARERR;
}

static DWORD pn(       /* Pseudo random number generator */
	DWORD pns   /* 0:Initialize, !0:Read */
)
{
	static DWORD lfsr;
	UINT n;


	if (pns) {
		lfsr = pns;
		for (n = 0; n < 32; n++) pn(0);
	}
	if (lfsr & 1) {
		lfsr >>= 1;
		lfsr ^= 0x80200003;
	}
	else {
		lfsr >>= 1;
	}
	return lfsr;
}


int test_diskio(
	BYTE pdrv,      /* Physical drive number to be checked (all data on the drive will be lost) */
	UINT ncyc,      /* Number of test cycles */
	DWORD* buff,    /* Pointer to the working buffer */
	UINT sz_buff    /* Size of the working buffer in unit of byte */
)
{
	UINT n, cc, ns;
	DWORD sz_drv, lba, lba2, sz_eblk, pns = 1;
	WORD sz_sect;
	BYTE* pbuff = (BYTE*)buff;
	BYTE buff2[4096];
	DSTATUS ds;
	DRESULT dr;


	printf("test_diskio(%u, %u, 0x%08X, 0x%08X)\n", pdrv, ncyc, (UINT)buff, sz_buff);

	if (sz_buff < FF_MAX_SS + 8) {
		printf("Insufficient work area to run the program.\n");
		return 1;
	}

	for (cc = 1; cc <= ncyc; cc++) {
		printf("**** Test cycle %u of %u start ****\n", cc, ncyc);

		printf(" disk_initalize(%u)", pdrv);
		ds = disk_initialize(pdrv);
		if (ds & STA_NOINIT) {
			printf(" - failed.\n");
			return 2;
		}
		else {
			printf(" - ok.\n");
		}

		printf("**** Get drive size ****\n");
		printf(" disk_ioctl(%u, GET_SECTOR_COUNT, 0x%08X)", pdrv, (UINT)&sz_drv);
		sz_drv = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_COUNT, &sz_drv);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 3;
		}
		if (sz_drv < 128) {
			printf("Failed: Insufficient drive size to test.\n");
			return 4;
		}
		printf(" Number of sectors on the drive %u is %lu.\n", pdrv, sz_drv);

#if FF_MAX_SS != FF_MIN_SS
		printf("**** Get sector size ****\n");
		printf(" disk_ioctl(%u, GET_SECTOR_SIZE, 0x%X)", pdrv, (UINT)&sz_sect);
		sz_sect = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_SIZE, &sz_sect);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 5;
		}
		printf(" Size of sector is %u bytes.\n", sz_sect);
#else
		sz_sect = FF_MAX_SS;
#endif

		printf("**** Get block size ****\n");
		printf(" disk_ioctl(%u, GET_BLOCK_SIZE, 0x%X)", pdrv, (UINT)&sz_eblk);
		sz_eblk = 0;
		dr = disk_ioctl(pdrv, GET_BLOCK_SIZE, &sz_eblk);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
		}
		if (dr == RES_OK || sz_eblk >= 2) {
			printf(" Size of the erase block is %lu sectors.\n", sz_eblk);
		}
		else {
			printf(" Size of the erase block is unknown.\n");
		}

		/* Single sector write test */
		printf("**** Single sector write test ****\n");
		lba = 0;
		for (n = 0, pn(pns); n < sz_sect; n++) buff2[n] = (BYTE)pn(0);
		printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
		dr = disk_write(pdrv, buff2, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 6;
		}
		printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 7;
		}
		memset(buff2, 0, sz_sect);
		printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
		dr = disk_read(pdrv, buff2, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 8;
		}
		for (n = 0, pn(pns); n < sz_sect && buff2[n] == (BYTE)pn(0); n++);
		if (n == sz_sect) {
			printf(" Read data matched.\n");
		}
		else {
			printf(" Read data differs from the data written.\n");
			return 10;
		}
		pns++;

		printf("**** Multiple sector write test ****\n");
		lba = 5; ns = sz_buff / sz_sect;
		if (ns > 4) ns = 4;
		if (ns > 1) {
			for (n = 0, pn(pns); n < (UINT)(sz_sect * ns); n++) pbuff[n] = (BYTE)pn(0);
			printf(" disk_write(%u, 0x%X, %lu, %u)", pdrv, (UINT)pbuff, lba, ns);
			dr = disk_write(pdrv, pbuff, lba, ns);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 11;
			}
			printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
			dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 12;
			}
			memset(pbuff, 0, sz_sect * ns);
			printf(" disk_read(%u, 0x%X, %lu, %u)", pdrv, (UINT)pbuff, lba, ns);
			dr = disk_read(pdrv, pbuff, lba, ns);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 13;
			}
			for (n = 0, pn(pns); n < (UINT)(sz_sect * ns) && pbuff[n] == (BYTE)pn(0); n++);
			if (n == (UINT)(sz_sect * ns)) {
				printf(" Read data matched.\n");
			}
			else {
				printf(" Read data differs from the data written.\n");
				return 14;
			}
		}
		else {
			printf(" Test skipped.\n");
		}
		pns++;

		printf("**** Single sector write test (unaligned buffer address) ****\n");
		lba = 5;
		for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n + 3] = (BYTE)pn(0);
		printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff + 3), lba);
		dr = disk_write(pdrv, pbuff + 3, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 15;
		}
		printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 16;
		}
		memset(pbuff + 5, 0, sz_sect);
		printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff + 5), lba);
		dr = disk_read(pdrv, pbuff + 5, lba, 1);
		if (dr == RES_OK) {
			printf(" - ok.\n");
		}
		else {
			printf(" - failed.\n");
			return 17;
		}
		for (n = 0, pn(pns); n < sz_sect && pbuff[n + 5] == (BYTE)pn(0); n++);
		if (n == sz_sect) {
			printf(" Read data matched.\n");
		}
		else {
			printf(" Read data differs from the data written.\n");
			return 18;
		}
		pns++;

		printf("**** 4GB barrier test ****\n");
		if (sz_drv >= 128 + 0x80000000 / (sz_sect / 2)) {
			lba = 6; lba2 = lba + 0x80000000 / (sz_sect / 2);
			for (n = 0, pn(pns); n < (UINT)(sz_sect * 2); n++) pbuff[n] = (BYTE)pn(0);
			printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
			dr = disk_write(pdrv, pbuff, lba, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 19;
			}
			printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff + sz_sect), lba2);
			dr = disk_write(pdrv, pbuff + sz_sect, lba2, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 20;
			}
			printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
			dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 21;
			}
			memset(pbuff, 0, sz_sect * 2);
			printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
			dr = disk_read(pdrv, pbuff, lba, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 22;
			}
			printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff + sz_sect), lba2);
			dr = disk_read(pdrv, pbuff + sz_sect, lba2, 1);
			if (dr == RES_OK) {
				printf(" - ok.\n");
			}
			else {
				printf(" - failed.\n");
				return 23;
			}
			for (n = 0, pn(pns); pbuff[n] == (BYTE)pn(0) && n < (UINT)(sz_sect * 2); n++);
			if (n == (UINT)(sz_sect * 2)) {
				printf(" Read data matched.\n");
			}
			else {
				printf(" Read data differs from the data written.\n");
				return 24;
			}
		}
		else {
			printf(" Test skipped.\n");
		}
		pns++;

		printf("**** Test cycle %u of %u completed ****\n\n", cc, ncyc);
	}

	return 0;
}