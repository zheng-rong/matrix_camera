#ifndef EpcsH
#define EpcsH EpcsH

//  Enhanced Programming Configuration Serial

#include "DeviceConfigureFrame.h"

//-----------------------------------------------------------------------------
//Operation codes for serial configuration devices
//-----------------------------------------------------------------------------
#define EpcsWrite_ENA		0x06	/* Write enable */
#define EpcsWrite_DIS		0x04	/* Write disable */
#define EpcsRead_STAT		0x05	/* Read status */
#define EpcsRead_BYTES		0x03	/* Read bytes */
#define EpcsFastRead_BYTES	0x0b	/* Fast Read bytes */
#define EpcsRead_ID			0xab	/* Read silicon id */
#define EpcsWrite_STAT		0x01	/* Write status */
#define EpcsWrite_BYTES		0x02	/* Write bytes */
#define EpcsErase_BULK		0xc7	/* Erase entire device */
#define EpcsErase_SECT		0xd8	/* Erase sector */

//-----------------------------------------------------------------------------
//Device status register bits
//-----------------------------------------------------------------------------
#define EPCS_STATUS_WIP		(1<<0)	/* Write in progress */
#define EPCS_STATUS_WEL		(1<<1)	/* Write enable latch */

#define EPCS_TIMEOUT		100	/* 100 msec timeout */

typedef struct SEpcsDevInfo{
	const char 	*name;		/* Device name */
	unsigned char	id;		/* Device silicon id */
	unsigned char	size;		/* Total size log2(bytes)*/
	unsigned char	num_sects;	/* Number of sectors */
	unsigned char	sz_sect;	/* Sector size log2(bytes) */
	unsigned char	sz_page;	/* Page size log2(bytes) */
	unsigned char   prot_mask;	/* Protection mask */
}TEpcsDevInfo;


typedef volatile struct SNiosSpi {
	unsigned	rxdata;		/* Rx data reg */
	unsigned	txdata;		/* Tx data reg */
	unsigned	status;		/* Status reg */
	unsigned	control;	/* Control reg */
	unsigned	reserved;	/* (master only) */
	unsigned	slaveselect;	/* SPI slave select mask (master only) */
}TNiosSpi;

#define REG_OFF_SPI_RXDATA	0
#define REG_OFF_SPI_TXDATA	0x4
#define REG_OFF_SPI_STATUS	0x8
#define REG_OFF_SPI_CONTROL	0xC

/* status register */
#define NIOS_SPI_ROE		(1 << 3)	/* rx overrun */
#define NIOS_SPI_TOE		(1 << 4)	/* tx overrun */
#define NIOS_SPI_TMT		(1 << 5)	/* tx empty */
#define NIOS_SPI_TRDY		(1 << 6)	/* tx ready */
#define NIOS_SPI_RRDY		(1 << 7)	/* rx ready */
#define NIOS_SPI_E			(1 << 8)	/* exception */

/* control register */
#define NIOS_SPI_IROE		(1 << 3)	/* rx overrun int ena */
#define NIOS_SPI_ITOE		(1 << 4)	/* tx overrun int ena */
#define NIOS_SPI_ITRDY		(1 << 6)	/* tx ready int ena */
#define NIOS_SPI_IRRDY		(1 << 7)	/* rx ready int ena */
#define NIOS_SPI_IE			(1 << 8)	/* exception int ena */
#define NIOS_SPI_SSO		(1 << 10)	/* override SS_n output */

typedef struct _RegisterAccess
{
	unsigned long Offset;
	unsigned long Data;
} TRegisterAccess;

class DeviceConfigureFrame;

class CEpcs
{
public:
	explicit	CEpcs( Device* pDev );
				~CEpcs();
	int WriteDataToFlash( unsigned char* flashdata, int bytes );
	void DeviceConfigure( );
	void Info( );
	void Erase( unsigned long start, unsigned long end );
	void BulkErase( ) { EpcsBulkErase( ); }
	void Protect( bool protect );
	void Read( unsigned char* flashdata, unsigned long off, unsigned long cnt );
	int Write( unsigned char* flashdata, unsigned long off, unsigned long cnt, bool verify );
	int Verify( unsigned char* flashdata, unsigned long off, unsigned long cnt );
	bool GetSPIDeviceInfo( TEpcsDevInfo &deviceInfo );
	bool IsProtected( ){ return Protected_; }
	void AttachParent( DeviceConfigureFrame* pParent ) { pParent_ = pParent; }
	void SectorInfo( );

	void DbOutput (const char * format, ...)
	{
#ifdef _DEBUG
		char string[256];
		va_list argptr;

		memset (string, 0, 256);
		va_start (argptr, format);
		vsprintf (string, format, argptr);
#ifdef linux
		printf( string );
#else
		OutputDebugStringA( string );
#endif
		va_end (argptr);
#else
		format = format; // remove compiler warning
#endif
	}

private:
	int EpcsCS( int assert );
	int EpcsTX( unsigned char c );
	int EpcsRX( );
	unsigned char EpcsBitRev( unsigned char c );
	void EpcsRcv( unsigned char *dst, int len );
	void EpcsRRcv( unsigned char *dst, int len );
	void EpcsSnd( unsigned char *src, int len );
	void EpcsRSnd( unsigned char *src, int len );
	void EpcsRSndEx( unsigned char *src, int len );
	void EpcsWrEnable( );
	unsigned char EpcsStatusRD( );
	void EpcsStatusWR( unsigned char status );
	TEpcsDevInfo* EpcsDevFind( );

	int EpcsCfgsz( );
	int EpcsErase( unsigned start, unsigned end );
	int EpcsRead( unsigned char* flashdata, unsigned long off, unsigned long cnt );
	int EpcsWrite( unsigned char* flashdata, unsigned long off, unsigned long cnt );
	int EpcsBulkErase( );
	int EpcsVerify( unsigned char* flashdata, unsigned long off, unsigned long cnt, unsigned long *err );
	int EpcsSectErased( int sect, unsigned *offset );
	int EpcsReset( );
	unsigned long ReadSPI( unsigned long off );
	void WriteSPI( unsigned long off, unsigned long data );

private:
	TNiosSpi* pEpcs_;
	TEpcsDevInfo* pDeviceInfo_;
	bool Protected_;
	Device* Dev_;
	Method* SPIRead_;
	Method* SPIWrite_;
	PropertyI* SPIOffset_;
	PropertyI* SPIData_;
	DeviceConfigureFrame* pParent_;
	int TXError_;
	int RXError_;
	int CSError_;
};

#endif //EpcsH
