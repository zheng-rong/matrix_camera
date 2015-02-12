//-----------------------------------------------------------------------------
#ifndef mvHYPERIONHandlerH
#define mvHYPERIONHandlerH mvHYPERIONHandlerH
//-----------------------------------------------------------------------------
#include "DeviceHandler.h"

//-----------------------------------------------------------------------------
class DeviceHandlerHYPERION : public DeviceHandler
//-----------------------------------------------------------------------------
{
public:
	DeviceHandlerHYPERION( mvIMPACT::acquire::Device* pDev ) : DeviceHandler(pDev) {}
	static DeviceHandler* Create( mvIMPACT::acquire::Device* pDev ) { return new DeviceHandlerHYPERION(pDev); }
	virtual bool SupportsFirmwareUpdate( void ) const { return true; }
	virtual bool SupportsDMABufferSizeUpdate( int* pCurrentDMASize_kB = 0 );
	virtual int UpdateFirmware( bool boSilentMode );
	virtual int UpdatePermanentDMABufferSize( bool boSilentMode );
	enum eFlashSelect
	{
		eFSSetI2CSwitchToDefaultFlash = 0,
		eFSSetI2CSwitchToUserFlash,
		eFSSetI2CSwitchOn,
		eFSSetI2CSwitchOffAndForceUserFlashSelect
	};
	void SelectFlashAccess( enum eFlashSelect access );
};

#endif // mvHYPERIONHandlerH
