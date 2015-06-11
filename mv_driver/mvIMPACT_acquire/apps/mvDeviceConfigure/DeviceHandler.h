//-----------------------------------------------------------------------------
#ifndef DeviceHandlerH
#define DeviceHandlerH DeviceHandlerH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

class DeviceConfigureFrame;

//-----------------------------------------------------------------------------
class DeviceHandler
//-----------------------------------------------------------------------------
{
	bool boSetIDSupported_;
protected:
	mvIMPACT::acquire::Device* pDev_;
	DeviceConfigureFrame* pParent_;
	wxString customFirmwarePath_;
	bool MessageToUser( const wxString& caption, const wxString& msg, bool boSilentMode, long style );
public:
	DeviceHandler( mvIMPACT::acquire::Device* pDev, bool boSetIDSupported = false ) : boSetIDSupported_(boSetIDSupported), pDev_(pDev), pParent_(0) {}
	virtual ~DeviceHandler() {}
	void AttachParent( DeviceConfigureFrame* pParent ) { pParent_ = pParent; }
	virtual bool SupportsFirmwareUpdate( void ) const = 0;
	virtual bool SupportsKernelDriverUpdate( bool& /*boNewerDriverAvailable*/, std::string& /*kernelDriverName*/ ) { return false; }
	bool SupportsSetID( void ) const { return boSetIDSupported_; }
	virtual bool GetIDFromUser( long& newID, const long minValue, const long maxValue );
	virtual bool SupportsDMABufferSizeUpdate( int* /* pCurrentDMASize_kB */ = 0 ) { return false; }
	virtual int UpdateFirmware( bool boSilentMode );
	virtual int UpdateKernelDriver( bool boSilentMode );
	virtual int UpdatePermanentDMABufferSize( bool /*boSilentMode*/ ) { return urFeatureUnsupported; }
	virtual void SetCustomFirmwarePath( const wxString& /* customFirmwarePath */ ) {}
	//-----------------------------------------------------------------------------
	enum TUpdateResult
	//-----------------------------------------------------------------------------
	{
		urOperationSuccessful,
		urFeatureUnsupported,
		urOperationCanceled,
		urInvalidFileSelection,
		urFileIOError,
		urDeviceAccessError
	};
};

#endif // DeviceHandlerH
