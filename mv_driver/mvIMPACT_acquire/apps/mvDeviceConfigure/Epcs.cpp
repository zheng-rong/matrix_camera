/*
 * Enhanced Programming Configuration Serial
 * (C) Copyright 2004, Psyent Corporation <www.psyent.com>
 * Scott McNutt <smcnutt@psyent.com>
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
#include <stdio.h>
#include "Epcs.h"
#include "wx/msgdlg.h"
#include "wx/string.h"
#include "Timeout.h"

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
#define SHORT_HELP\
	"epcs    - read/write Cyclone EPCS configuration device.\n"

#define LONG_HELP\
	"\n"\
	"epcs erase start [end]\n"\
	"    - erase sector start or sectors start through end.\n"\
	"epcs info\n"\
	"    - display EPCS device information.\n"\
	"epcs protect on | off\n"\
	"    - turn device protection on or off.\n"\
	"epcs read addr offset count\n"\
	"    - read count bytes from offset to addr.\n"\
	"epcs write addr offset count\n"\
	"    - write count bytes to offset from addr.\n"\
	"epcs verify addr offset count\n"\
	"    - verify count bytes at offset from addr.\n"


//static struct epcs_devinfo_t devinfo[] = {
//	{ "EPCS1 ", 0x10, 17, 4, 15, 8, 0x0c },
//	{ "EPCS4 ", 0x12, 19, 8, 16, 8, 0x1c },
//	{ 0, 0, 0, 0, 0, 0 }
//};

TEpcsDevInfo DeviceInfo[] = {
	{ "M25p40Flash ", 0x12 , 22, 8, 19, 8, 0x1c },
	{ "M25p16 ", 0x14 , 24, 32, 19, 8, 0x1c },
	{ 0, 0, 0, 0, 0, 0, 0 }
};


//-----------------------------------------------------------------------------
CEpcs::CEpcs( Device* pDev ) : Dev_(pDev), TXError_(0), RXError_(0), CSError_(0)
//-----------------------------------------------------------------------------
{
	ComponentLocator locator(Dev_->hDev());
	SPIRead_ = new Method(locator.findComponent( "ReadSPI@i" ));
	SPIWrite_ = new Method(locator.findComponent( "WriteSPI@i" ));
	SPIOffset_ = new PropertyI(locator.findComponent( "SPIOffset" ));
	SPIData_ = new PropertyI(locator.findComponent( "SPIData" ));
	Method configSPI(locator.findComponent( "ConfigSPI@ii" ));
	configSPI.call("1");
}

//-----------------------------------------------------------------------------
CEpcs::~CEpcs()
//-----------------------------------------------------------------------------
{
	delete SPIRead_;
	delete SPIWrite_;
	delete SPIOffset_;
	delete SPIData_;
	ComponentLocator locator(Dev_->hDev());
	Method configSPI(locator.findComponent( "ConfigSPI@ii" ));
	configSPI.call("0");
}

//-----------------------------------------------------------------------------
// Device access
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
unsigned long CEpcs::ReadSPI( unsigned long off )
//-----------------------------------------------------------------------------
{
	SPIOffset_->write( off );
	return SPIRead_->call();
}

//-----------------------------------------------------------------------------
void CEpcs::WriteSPI( unsigned long off, unsigned long data )
//-----------------------------------------------------------------------------
{
	SPIOffset_->write( off );
	SPIData_->write( data );
	SPIWrite_->call();
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsCS( int assert )
//-----------------------------------------------------------------------------
{
	CTimeout timeout( EPCS_TIMEOUT );
	unsigned long tmp;

	CSError_ = 0;
	if (assert) {
		tmp = ReadSPI( REG_OFF_SPI_CONTROL );
		WriteSPI( REG_OFF_SPI_CONTROL, tmp | NIOS_SPI_SSO );
	} else {
		/* Let all bits shift out */
		while( (ReadSPI( REG_OFF_SPI_STATUS ) & NIOS_SPI_TMT) == 0 )
		{
			if( timeout.Elapsed() )
			{
				CSError_ = -1;
				return CSError_;
			}
			timeout.WaitTimeout(1);
		}
		tmp = ReadSPI( REG_OFF_SPI_CONTROL );
		WriteSPI( REG_OFF_SPI_CONTROL, tmp & ~NIOS_SPI_SSO );
	}
	return CSError_;
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsTX( unsigned char c )
//-----------------------------------------------------------------------------
{
	CTimeout timeout( EPCS_TIMEOUT );

	TXError_ = 0;
	while ((ReadSPI( REG_OFF_SPI_STATUS ) & NIOS_SPI_TRDY) == 0)
	{
		if( timeout.Elapsed() )
		{
			TXError_ = -1;
			return TXError_;
		}
		timeout.WaitTimeout(1);
	}
	WriteSPI( REG_OFF_SPI_TXDATA, c );
	return TXError_;
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsRX( void )
//-----------------------------------------------------------------------------
{
	CTimeout timeout( EPCS_TIMEOUT );

	RXError_ = 0;
	while ((ReadSPI( REG_OFF_SPI_STATUS ) & NIOS_SPI_RRDY) == 0)
	{
		if( timeout.Elapsed() )
		{
			RXError_ = -1;
			return RXError_;
		}
		timeout.WaitTimeout(1);
	}
	return ReadSPI( REG_OFF_SPI_RXDATA );
}

unsigned char bitrev[] = {
	0x00, 0x08, 0x04, 0x0c, 0x02, 0x0a, 0x06, 0x0e,
	0x01, 0x09, 0x05, 0x0d, 0x03, 0x0b, 0x07, 0x0f
};

//-----------------------------------------------------------------------------
unsigned char CEpcs::EpcsBitRev( unsigned char c )
//-----------------------------------------------------------------------------
{
	unsigned char val;

	val  = bitrev[c>>4];
	val |= bitrev[c & 0x0f]<<4;
	return (val);
}

//-----------------------------------------------------------------------------
void CEpcs::EpcsRcv( unsigned char *dst, int len )
//-----------------------------------------------------------------------------
{
	while (len--) {
		EpcsTX (0);
		*dst++ = (unsigned char)EpcsRX ();
	}
}

//-----------------------------------------------------------------------------
void CEpcs::EpcsRRcv( unsigned char *dst, int len )
//-----------------------------------------------------------------------------
{
	while (len--) {
		EpcsTX (0);
		*dst++ = EpcsBitRev ((unsigned char)EpcsRX ());
	}
}

//-----------------------------------------------------------------------------
void CEpcs::EpcsSnd( unsigned char *src, int len )
//-----------------------------------------------------------------------------
{
	while (len--) {
		EpcsTX (*src++);
		EpcsRX ();
	}
}

//-----------------------------------------------------------------------------
void CEpcs::EpcsRSnd( unsigned char *src, int len )
//-----------------------------------------------------------------------------
{
	while (len--) {
		EpcsTX (EpcsBitRev (*src++));
		EpcsRX ();
	}
}

//-----------------------------------------------------------------------------
void CEpcs::EpcsRSndEx( unsigned char *src, int len )
//-----------------------------------------------------------------------------
{
	while (len--) 
		EpcsTX (EpcsBitRev (*src++));
}

//-----------------------------------------------------------------------------
void CEpcs::EpcsWrEnable( )
//-----------------------------------------------------------------------------
{
	EpcsCS (1);
	EpcsTX (EpcsWrite_ENA);
	EpcsRX ();
	EpcsCS (0);
}

//-----------------------------------------------------------------------------
unsigned char CEpcs::EpcsStatusRD( )
//-----------------------------------------------------------------------------
{
	unsigned char status;

	EpcsCS (1);
	EpcsTX (EpcsRead_STAT);
	EpcsRX ();
	EpcsTX (0);
	status = (unsigned char)EpcsRX ();
	EpcsCS (0);
	return (status);
}

//-----------------------------------------------------------------------------
void CEpcs::EpcsStatusWR( unsigned char status )
//-----------------------------------------------------------------------------
{
	EpcsWrEnable ();
	EpcsCS (1);
	EpcsTX (EpcsWrite_STAT);
	EpcsRX ();
	EpcsTX (status);
	EpcsRX ();
	EpcsCS (0);
	return;
}

/***********************************************************************
 * Device information
 ***********************************************************************/

//struct epcs_devinfo_t devinfo[] = {
//	{ "EPCS1 ", 0x10, 17, 4, 15, 8, 0x0c },
//	{ "EPCS4 ", 0x12, 19, 8, 16, 8, 0x1c },
//	{ 0, 0, 0, 0, 0, 0 }
//};
//typedef struct epcs_devinfo_t {
//	const char 	*name;		/* Device name */
//	unsigned char	id;		/* Device silicon id */
//	unsigned char	size;		/* Total size log2(bytes)*/
//	unsigned char	num_sects;	/* Number of sectors */
//	unsigned char	sz_sect;	/* Sector size log2(bytes) */
//	unsigned char	sz_page;	/* Page size log2(bytes) */
//	unsigned char   prot_mask;	/* Protection mask */
//}epcs_devinfo_t;


//-----------------------------------------------------------------------------
int CEpcs::EpcsReset( )
//-----------------------------------------------------------------------------
{
	/* When booting from an epcs controller, the epcs bootrom
	 * code may leave the slave select in an asserted state.
	 * This causes two problems: (1) The initial epcs access
	 * will fail -- not a big deal, and (2) a software reset
	 * will cause the bootrom code to hang since it does not
	 * ensure the select is negated prior to first access -- a
	 * big deal. Here we just negate chip select and everything
	 * gets better :-)
	 */
	EpcsCS (0); /* Negate chip select */
	return (0);
}

//-----------------------------------------------------------------------------
TEpcsDevInfo* CEpcs::EpcsDevFind( void )
//-----------------------------------------------------------------------------
{
	unsigned char buf[4];
	unsigned char id;
	int i;

	/* Read silicon id requires 3 "dummy bytes" before it's put on the wire. */
	buf[0] = EpcsRead_ID;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;

	EpcsCS (1);
	EpcsSnd (buf,4);
	EpcsRcv(buf,1);
	if (EpcsCS (0) == -1)
	{
		DbOutput( "error %s EpcsCS returns -1\n", __FUNCTION__ );
		return 0;
	}
	id = buf[0];

	/* Find the info struct */
	i = 0;
	while (DeviceInfo[i].name) {
		if (id == DeviceInfo[i].id)
		{
			DbOutput( "found device %s\n", DeviceInfo[i].name );
			return &DeviceInfo[i];
		}
		i++;
	}
	return 0;
}

//-----------------------------------------------------------------------------
/***********************************************************************
 * Misc Utilities
 ***********************************************************************/
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
int CEpcs::EpcsCfgsz( )
//-----------------------------------------------------------------------------
{
	int sz = 0;
	unsigned char buf[128];
	unsigned char *p;

	/* Read in the first 128 bytes of the device */
	buf[0] = EpcsRead_BYTES;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;

	EpcsCS (1);
	EpcsSnd (buf,4);
	EpcsRRcv(buf, sizeof(buf));
	EpcsCS (0);

	/* Search for the starting 0x6a which is followed by the
	 * 4-byte 'register' and 4-byte bit-count.
	 */
	p = buf;
	while (p < buf + sizeof(buf)-8) {
		if ( *p == 0x6a ) {
			/* Point to bit count and extract */
			p += 5;
			sz = *p++;
			sz |= *p++ << 8;
			sz |= *p++ << 16;
			sz |= *p++ << 24;
			/* Convert to byte count */
			sz += 7;
			sz >>= 3;
		} else if (*p == 0xff) {
			/* 0xff is ok ... just skip */
			p++;
			continue;
		} else {
			/* Not 0xff or 0x6a ... something's not
			 * right ... report 'unknown' (sz=0).
			 */
			break;
		}
	}
	return (sz);
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsErase( unsigned start, unsigned end )
//-----------------------------------------------------------------------------
{
	unsigned off, sectsz;
	unsigned char buf[4];

	/* Erase the requested sectors. An address is required
	 * that lies within the requested sector -- we'll just
	 * use the first address in the sector.
	 */
	DbOutput( " epcs erasing sector %d to %d\n", start, end+1 );
	sectsz = (1 << pDeviceInfo_->sz_sect);
	while (start <= end) {
		off = start * sectsz;
		start++;

		buf[0] = EpcsErase_SECT;
		buf[1] = (unsigned char)(off >> 16);
		buf[2] = (unsigned char)(off >> 8);
		buf[3] = (unsigned char)off;

		EpcsWrEnable ();
		EpcsCS (1);
		EpcsSnd (buf,4);
		EpcsCS (0);

		DbOutput( " ." ); /* Some user feedback */

		/* Wait for erase to complete */
		while (EpcsStatusRD() & EPCS_STATUS_WIP)
			;
	}
	DbOutput( "  done.\n" );
	return (0);
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsRead( unsigned char* flashdata, unsigned long off, unsigned long cnt )
//-----------------------------------------------------------------------------
{
	unsigned char buf[4];

	buf[0] = EpcsRead_BYTES;
	buf[1] = (unsigned char)(off >> 16);
	buf[2] = (unsigned char)(off >> 8);
	buf[3] = (unsigned char)off;

	EpcsCS (1);
	EpcsSnd (buf,4);
	EpcsRRcv(flashdata, cnt);
	EpcsCS (0);

	return (0);
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsWrite( unsigned char* flashdata, unsigned long off, unsigned long cnt )
//-----------------------------------------------------------------------------
{
	unsigned long wrcnt, i;
	unsigned pgsz;
	unsigned char buf[4];

	pgsz = (1<<pDeviceInfo_->sz_page);
	while (cnt) {
		if (off % pgsz)
			wrcnt = pgsz - (off % pgsz);
		else
			wrcnt = pgsz;
		wrcnt = (wrcnt > cnt) ? cnt : wrcnt;

		buf[0] = EpcsWrite_BYTES;
		buf[1] = (unsigned char)(off >> 16);
		buf[2] = (unsigned char)(off >> 8);
		buf[3] = (unsigned char)off;

		for( i = 0; i < wrcnt; i++ )
		{
			if( flashdata[i] != 0xff )
			{
				//DbOutput( " %s off %d, cnt %d\n", __FUNCTION__, off, cnt);
				EpcsWrEnable ();
				EpcsCS (1);
				EpcsSnd (buf,4);
				EpcsRSndEx (flashdata, wrcnt);
				//EpcsRSnd (flashdata, wrcnt);
				EpcsCS (0);
				/* Wait for write to complete */
				while (EpcsStatusRD() & EPCS_STATUS_WIP)
					;					
				break;
			}
		}

		/* Wait for write to complete */
		while (EpcsStatusRD() & EPCS_STATUS_WIP)
			;

		cnt -= wrcnt;
		off += wrcnt;
		flashdata += wrcnt;

		if( TXError_ != 0 || RXError_ != 0 || CSError_ != 0 )
			return (-1);
	}
	return (0);
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsBulkErase( )
//-----------------------------------------------------------------------------
{
	unsigned char buf[4];

	DbOutput( " > %s", __FUNCTION__ );
	buf[0] = EpcsErase_BULK;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;

	EpcsWrEnable ();
	EpcsCS (1);
	EpcsSnd (buf,1);
	EpcsCS (0);

	/* Wait for write to complete */
	while (EpcsStatusRD() & EPCS_STATUS_WIP)
		;

	DbOutput( " < %s", __FUNCTION__ );
	return (0);
}


//-----------------------------------------------------------------------------
int CEpcs::EpcsVerify( unsigned char* flashdata, unsigned long off, unsigned long cnt, unsigned long *err )
//-----------------------------------------------------------------------------
{
	unsigned long i, rdcnt;
	unsigned char buf[256];
	unsigned char *start,*end;

	start = end = flashdata;
	while (cnt) {
		rdcnt = (cnt>sizeof(buf)) ? sizeof(buf) : cnt;
		EpcsRead (buf, off, rdcnt);
		for (i=0; i<rdcnt; i++) {
			if (*end != buf[i]) {
				*err = static_cast<unsigned long>(end - start);
				return(-1);
			}
			end++;
		}
		cnt -= rdcnt;
		off += rdcnt;
	}
	return (0);
}

//-----------------------------------------------------------------------------
int CEpcs::EpcsSectErased(int sect, unsigned *offset )
//-----------------------------------------------------------------------------
{
	unsigned char buf[128];
	unsigned off, end;
	unsigned sectsz;
	unsigned int i;

	sectsz = (1 << pDeviceInfo_->sz_sect);
	off = sectsz * sect;
	end = off + sectsz;

	while (off < end) {
		EpcsRead( buf, off, sizeof(buf) );
		for (i=0; i < sizeof(buf); i++) {
			if (buf[i] != 0xff) {
				*offset = off + i;
				return (0);
			}
		}
		off += sizeof(buf);
	}
	return (1);
}



/***********************************************************************
 * Commands
 ***********************************************************************/

//-----------------------------------------------------------------------------
void CEpcs::Info( )
//-----------------------------------------------------------------------------
{
	unsigned char stat;
	unsigned tmp;

	if( !pDeviceInfo_ )
		return;
	/* Basic device info */
	DbOutput( " > %s: %d kbit (%d sectors x %d kbytes," " %d bytes/page)\n", pDeviceInfo_->name, 1 << (pDeviceInfo_->size-10), pDeviceInfo_->num_sects, 1 << (pDeviceInfo_->sz_sect-10), 1 << pDeviceInfo_->sz_page  );
	/* Status -- for now protection is all-or-nothing */
	stat = EpcsStatusRD();
	Protected_ = ((stat & pDeviceInfo_->prot_mask) > 0);
	DbOutput( " status: 0x%02x (WIP:%d, WEL:%d, PROT:%s)\n", stat, (stat & EPCS_STATUS_WIP) ? 1 : 0, (stat & EPCS_STATUS_WEL) ? 1 : 0, Protected_ ? "on" : "off"  );
	/* Configuration  */
	tmp = EpcsCfgsz ();
	if (tmp) {
		DbOutput( " config: 0x%06x (%d) bytes\n", tmp, tmp  );
	} else {
		DbOutput( " config: unknown\n"  );
	}

	/*
	//Sector info
	{
		int erased;
		for (int i = 0; i < pDeviceInfo_->num_sects; i++) {
			erased = EpcsSectErased (i, &tmp );
			DbOutput( "      %d: %06x ", i, i*(1 << pDeviceInfo_->sz_sect)  );
			if (erased)
				DbOutput( " erased\n" );
			else
				DbOutput( " data @ 0x%06x\n", tmp );
		}
	}*/
	return;
}

//-----------------------------------------------------------------------------
void CEpcs::SectorInfo( )
//-----------------------------------------------------------------------------
{
	unsigned int tmp;
	if( !pDeviceInfo_ )
		return;
	//Sector info
	{
		int erased;
		int maxnumber_sects = pDeviceInfo_->num_sects < 2 ? 2 : pDeviceInfo_->num_sects;
		for (int i = 0; i < maxnumber_sects; i++) {
			erased = EpcsSectErased (i, &tmp );
			DbOutput( "      %d: %06x ", i, i*(1 << pDeviceInfo_->sz_sect)  );
			if (erased)
				DbOutput( " erased\n" );
			else
				DbOutput( " data @ 0x%06x\n", tmp );
		}
	}
}

//-----------------------------------------------------------------------------
void CEpcs::Erase( unsigned long start, unsigned long end )
//-----------------------------------------------------------------------------
{
	if( !pDeviceInfo_ )
		return;
	if ((start >= pDeviceInfo_->num_sects) || (start > end)) {
		DbOutput( " epcs: invalid sector range: [%d:%d]\n", start, end  );
		return;
	}
	EpcsErase (start, end);
	return;
}

//-----------------------------------------------------------------------------
void CEpcs::Protect( bool protect )
//-----------------------------------------------------------------------------
{
	unsigned char stat;

	/* For now protection is all-or-nothing to keep things
	 * simple. The protection bits don't map in a linear
	 * fashion ... and we would rather protect the bottom
	 * of the device since it contains the config data and
	 * leave the top unprotected for app use. But unfortunately
	 * protection works from top-to-bottom so it does
	 * really help very much from a software app point-of-view.
	 */
	if (!pDeviceInfo_)
		return;
	/* Protection on/off is just a matter of setting/clearing
	 * all protection bits in the status register.
	 */
	stat = EpcsStatusRD ();
	if( protect == true )
		stat |= pDeviceInfo_->prot_mask;
	else
		stat &= ~pDeviceInfo_->prot_mask;

	EpcsStatusWR (stat);
	/* Wait for protect to complete */
	while (EpcsStatusRD() & EPCS_STATUS_WIP)
		;
	return;
}

//-----------------------------------------------------------------------------
void CEpcs::Read( unsigned char* flashdata, unsigned long off, unsigned long cnt )
//-----------------------------------------------------------------------------
{
	unsigned long sz;

	if( !pDeviceInfo_ )
		return;

	sz = 1 << pDeviceInfo_->size;
	if (off > sz) {
		DbOutput( " offset is greater than device size" "... aborting.\n" );
		return;
	}
	if ((off + cnt) > sz) {
		DbOutput( " request exceeds device size" "... truncating.\n" );
		cnt = sz - off;
	}
	DbOutput( " epcs: read %x <- %06lx (0x%lx bytes)\n", *flashdata, off, cnt );
	EpcsRead (flashdata, off, cnt);
	return;
}

//-----------------------------------------------------------------------------
int CEpcs::Write( unsigned char* flashdata, unsigned long off, unsigned long cnt, bool verify )
//-----------------------------------------------------------------------------
{
	unsigned long sz;
	unsigned long err;
	int result;

	if( !pDeviceInfo_ )
		return -1;
	if ((EpcsStatusRD() & pDeviceInfo_->prot_mask) != 0) {
		DbOutput( "epcs: device protected.\n" );
		return -2;
	}

	sz = 1 << pDeviceInfo_->size;
	if (off > sz) {
		DbOutput( " offset is greater than device size" "... aborting.\n" );
		return -3;
	}
	if ((off + cnt) > sz) {
		DbOutput( " request exceeds device size" "... truncating.\n" );
		cnt = sz - off;
	}
	DbOutput( " > %s %x -> %06lx (0x%lx bytes)\n", __FUNCTION__, *flashdata, off, cnt );
	result = EpcsWrite( flashdata, off, cnt );
	if( verify == true )
	{
		DbOutput( " %s start verify\n",__FUNCTION__ );
		if (EpcsVerify (flashdata, off, cnt, &err) != 0)
		{
			DbOutput( " %s error at offset %06lx\n", __FUNCTION__, err );
			result = -4;
		}
		else
		{
			DbOutput( " %s %x -> %06lx (0x%lx bytes) verify ok\n", __FUNCTION__, *flashdata, off, cnt );
			result = 0;
		}
	}
	DbOutput( " < %s result %d\n", __FUNCTION__, result );
	return result;	
}

//-----------------------------------------------------------------------------
int CEpcs::Verify( unsigned char* flashdata, unsigned long off, unsigned long cnt )
//-----------------------------------------------------------------------------
{
	unsigned long sz;
	unsigned long err;

	if( !pDeviceInfo_ )
		return -101;
	/*if (argc < 5) {
		DbOutput( " USAGE: epcs verify addr offset count\n" );
		return;
	}*/
	sz = 1 << pDeviceInfo_->size;
	if (off > sz) {
		DbOutput( " offset is greater than device size" "... aborting\n" );
		return -2;
	}
	if ((off + cnt) > sz) 
	{
		DbOutput( " request exceeds device size" "... truncating\n" );
		cnt = sz - off;
	}
	DbOutput( " %s %x -> %06lx (0x%lx bytes)\n", __FUNCTION__, *flashdata, off, cnt );
	if (EpcsVerify (flashdata, off, cnt, &err) != 0)
	{
		DbOutput( " epcs: verify error at offset %06lx\n", err );
		return -1;
	}
	else
	{
		DbOutput( " %s %x -> %06lx (0x%lx bytes) verify ok\n", __FUNCTION__, *flashdata, off, cnt );
		return 0;
	}
}

//-----------------------------------------------------------------------------
bool CEpcs::GetSPIDeviceInfo( TEpcsDevInfo &deviceInfo )
//-----------------------------------------------------------------------------
{
	if( !pDeviceInfo_ )
		return false;

	memcpy( &deviceInfo, pDeviceInfo_, sizeof(TEpcsDevInfo) );
	return true;
}

//-----------------------------------------------------------------------------
void CEpcs::DeviceConfigure( )
//-----------------------------------------------------------------------------
{
	pDeviceInfo_ = EpcsDevFind ();
	if (!pDeviceInfo_ )
		return ;
	Info( );
}


//-----------------------------------------------------------------------------
int CEpcs::WriteDataToFlash( unsigned char* flashdata, int bytes )
//-----------------------------------------------------------------------------
{
	if (!pDeviceInfo_ )
		return -101;
	//EpcsErase (0, dev->num_sects-1); //erase complete flah
	//EpcsBulkErase ();
	return Write( flashdata, 0, bytes, false );
}

