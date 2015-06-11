//-----------------------------------------------------------------------------
#ifndef DeviceHandlerBlueDeviceH
#define DeviceHandlerBlueDeviceH DeviceHandlerBlueDeviceH
//-----------------------------------------------------------------------------
#include "DeviceHandler.h"
#include "PackageDescriptionParser.h"

//-----------------------------------------------------------------------------
class DeviceHandlerBlueDevice : public DeviceHandler
//-----------------------------------------------------------------------------
{
	enum TProductGroup
	{
		pgUnknown,
		pgBlueCOUGAR_P,
		pgBlueCOUGAR_S,
		pgBlueCOUGAR_X,
		pgBlueFOX3,
		pgBlueLYNX_M7
	};
	TProductGroup product_;
	wxString productStringForFirmwareUpdateCheck_;
	wxString firmwareUpdateFileName_;
	wxString firmwareUpdateFolder_;
	int CheckForIncompatibleFirmwareVersions_BlueCOUGAR_X( bool boSilentMode, const wxString& serial, const FileEntryContainer& fileEntries, const wxString& selection, const Version& currentFirmwareVersion );
	TUpdateResult DoFirmwareUpdate_BlueCOUGAR_X( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize );
	TUpdateResult DoFirmwareUpdate_BlueFOX3( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize );
	bool ExtractFileVersion( const wxString& fileName, Version& fileVersion ) const;
	bool GetFileFromArchive( const wxString& firmwareFileAndPath, const char* pArchive, size_t archiveSize, const wxString& filename, auto_array_ptr<char>& data );
	bool IsBlueCOUGAR_X( mvIMPACT::acquire::Device* pDev );
	bool IsBlueFOX3( mvIMPACT::acquire::Device* pDev );
	int UpdateCOUGAR_SDevice( bool boSilentMode );
	int UpdateCOUGAR_XAndFOX3Device( bool boSilentMode );
	int UpdateLYNX_M7AndCOUGAR_PDevice( const wxString& updateFileName, const wxString& fileExtension, bool boSilentMode );
	int UploadFile( const wxString& fullPath, const wxString& descriptionFile );
public:
	DeviceHandlerBlueDevice( mvIMPACT::acquire::Device* pDev );
	static DeviceHandler* Create( mvIMPACT::acquire::Device* pDev ) { return new DeviceHandlerBlueDevice(pDev); }
	virtual bool GetIDFromUser( long& newID, const long minValue, const long maxValue );
	virtual bool SupportsFirmwareUpdate( void ) const;
	virtual int UpdateFirmware( bool boSilentMode );
	virtual void SetCustomFirmwarePath( const wxString& customFirmwarePath );
};

int        CompareFileVersion( const wxString& first, const wxString& second );
wxString   ExtractVersionNumber( const wxString& s );
bool       GetNextNumber( wxString& str, long& number );
int64_type MACAddressFromString( const std::string& MAC );

#endif // DeviceHandlerBlueDeviceH
