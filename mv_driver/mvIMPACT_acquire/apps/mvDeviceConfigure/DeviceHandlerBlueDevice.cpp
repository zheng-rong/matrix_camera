//-----------------------------------------------------------------------------
#include <apps/Common/wxAbstraction.h>
#include "DeviceConfigureFrame.h"
#include "DeviceHandlerBlueDevice.h"
#include <memory>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam_FileStream.h>
#include <wx/choicdlg.h>
#include <wx/dir.h>
#include <wx/ffile.h>
#include <wx/filedlg.h>
#include <wx/mstream.h>
#include <wx/utils.h>
#include <wx/zipstrm.h>

//=============================================================================
//=================== internal helper functions and structs ===================
//=============================================================================
//-----------------------------------------------------------------------------
struct UpdateFeatures
//-----------------------------------------------------------------------------
{
	Device* pDev_;
	PropertyS sourceFileName_;
	PropertyS destinationFileName_;
	PropertyI transferMode_;
	PropertyS transferBuffer_;
	Method upload_;
	Method download_;
	Method install_;
	Method updateFirmware_;
	PropertyS lastResult_;
	void bindFeatures( void )
	{
		DeviceComponentLocator locator(pDev_, dltSystemSettings, "FileExchange");
		locator.bindComponent( sourceFileName_, "SourceFileName" );
		locator.bindComponent( destinationFileName_, "DestinationFileName" );
		locator.bindComponent( transferMode_, "TransferMode" );
		locator.bindComponent( transferBuffer_, "TransferBuffer" );
		locator.bindComponent( upload_, "DoFileUpload@i" );
		locator.bindComponent( download_, "DoFileDownload@i" );
		locator.bindComponent( install_, "DoInstallFile@i" );
		locator.bindComponent( updateFirmware_, "DoFirmwareUpdate@i" );
		locator.bindComponent( lastResult_, "LastResult" );
	}
	explicit UpdateFeatures( Device* pDev ) : pDev_(pDev)
	{
		bindFeatures();
	}
};

//-----------------------------------------------------------------------------
class SystemSettingsGenTL : public SystemSettings
//-----------------------------------------------------------------------------
{
public:
	explicit SystemSettingsGenTL( Device* pDev ) : SystemSettings(pDev),
		featurePollingEnable(), featurePollingInterval_ms()
	{
		DeviceComponentLocator locator(pDev, dltSystemSettings);
		locator.bindComponent( featurePollingEnable, "FeaturePollingEnable" );
		locator.bindComponent( featurePollingInterval_ms, "FeaturePollingInterval_ms" );
	}
	PropertyIBoolean featurePollingEnable;
	PropertyI featurePollingInterval_ms;
};

//-----------------------------------------------------------------------------
template<class _Elem, class _Traits, class _Ax, class _AVx>
typename std::vector<std::basic_string<_Elem, _Traits, _AVx> >::size_type split(
	/// The string to split
	const std::basic_string<_Elem,_Traits,_Ax>& str,
	/// A string defining a separator
	const std::basic_string<_Elem,_Traits,_Ax>& separator,
	/// A vector for the resulting strings
	std::vector<std::basic_string<_Elem,_Traits,_Ax>, _AVx >& v )
//-----------------------------------------------------------------------------
{
	v.clear();
	typename std::basic_string<_Elem, _Traits, _Ax>::size_type start = 0, end = 0;
	std::basic_string<_Elem, _Traits, _Ax> param, key, value;
	while( ( start = str.find_first_not_of( separator, start )) != std::basic_string<_Elem, _Traits, _Ax>::npos )
	{
		end = str.find_first_of( separator, start );
		v.push_back( ( end == std::basic_string<_Elem, _Traits, _Ax>::npos ) ? str.substr( start ) : str.substr( start, end - start ) );
		start = end;
	}
	return v.size();
}

//-----------------------------------------------------------------------------
/// Highest file number first!
int CompareFileVersion( const wxString& first, const wxString& second )
//-----------------------------------------------------------------------------
{
	Version firstVersion(VersionFromString( ExtractVersionNumber( first ) ));
	Version secondVersion(VersionFromString( ExtractVersionNumber( second ) ));
	if( firstVersion < secondVersion )
	{
		return 1;
	}
	else if( firstVersion > secondVersion )
	{
		return -1;
	}
	return 0;
}

//-----------------------------------------------------------------------------
wxString ExtractVersionNumber( const wxString& s )
//-----------------------------------------------------------------------------
{
	//%s(%s, file version: %d.%d)
	static const wxString TOKEN(wxT("version: "));
	int pos = s.Find( TOKEN.c_str() );
	if( pos == wxNOT_FOUND )
	{
		return wxString();
	}
	wxString version(s.Mid( pos + TOKEN.Length() ));
	int posVersionEND = version.Find( wxT(')') );
	return ( ( posVersionEND == wxNOT_FOUND ) ? version : version.Mid( 0, posVersionEND ) );
}

//-----------------------------------------------------------------------------
bool GetNextNumber( wxString& str, long& number )
//-----------------------------------------------------------------------------
{
	wxString numberString = str.BeforeFirst( wxT('_') );
	str = str.AfterFirst( wxT('_') );
	return numberString.ToLong( &number );
}

//-----------------------------------------------------------------------------
int64_type MACAddressFromString( const std::string& MAC )
//-----------------------------------------------------------------------------
{
	int64_type result = 0;
	std::vector<std::string> v;
	std::vector<std::string>::size_type cnt = split( MAC, std::string(":"), v );
	if( cnt != 6 )
	{
		// invalid string format
		return 0;
	}
	for( std::vector<std::string>::size_type i=0; i<cnt; i++ )
	{
		unsigned int val;
		sscanf( v[i].c_str(), "%x", &val );
		result = result | ( static_cast<int64_type>(val) << ( 8 * ( cnt - 1 - i ) ) );
	}
	return result;
}

//=============================================================================
//=================== implementation DeviceHandlerBlueDevice ==================
//=============================================================================
//-----------------------------------------------------------------------------
DeviceHandlerBlueDevice::DeviceHandlerBlueDevice( mvIMPACT::acquire::Device* pDev ) : DeviceHandler(pDev, true), product_(pgUnknown)
//-----------------------------------------------------------------------------
{
	ConvertedString product(pDev_->product.read());
	if( product.Find( wxT("mvBlueCOUGAR-P") ) != wxNOT_FOUND )
	{
		product_ = pgBlueCOUGAR_P;
	}
	else if( product.Find( wxT("mvBlueCOUGAR-S") ) != wxNOT_FOUND )
	{
		product_ = pgBlueCOUGAR_S;
	}
	else if( IsBlueCOUGAR_X( pDev ) )
	{
		product_ = pgBlueCOUGAR_X;
	}
	else if( IsBlueFOX3( pDev ) )
	{
		product_ = pgBlueFOX3;
	}
	else if( product.Find( wxT("mvBlueLYNX-M7") ) != wxNOT_FOUND )
	{
		product_ = pgBlueLYNX_M7;
	}

	switch( product_ )
	{
	case pgBlueCOUGAR_P:
		firmwareUpdateFileName_ = wxString(wxT("mvBlueCOUGAR-P_Update.tgz"));
		productStringForFirmwareUpdateCheck_ = wxString(wxT("mvBlueCOUGAR-P"));
		break;
	case pgBlueCOUGAR_S:
		firmwareUpdateFileName_ = product;
		if( pDev->firmwareVersion.read() >= 0x10100 )
		{
			// this is HW revision 1, which means it needs a different firmware
			firmwareUpdateFileName_.Append( wxT("_R001") );
		}
		firmwareUpdateFileName_.Append( wxT("_Update.fpg") );
		productStringForFirmwareUpdateCheck_ = firmwareUpdateFileName_;
		break;
	case pgBlueCOUGAR_X:
		firmwareUpdateFileName_ = wxString(wxT("mvBlueCOUGAR-X_Update.mvu"));
		productStringForFirmwareUpdateCheck_ = wxString(wxT("mvBlueCOUGAR-X"));
		break;
	case pgBlueFOX3:
		firmwareUpdateFileName_ = wxString(wxT("mvBlueFOX3_Update.mvu"));
		productStringForFirmwareUpdateCheck_ = wxString(wxT("mvBlueFOX3"));
		break;
	case pgBlueLYNX_M7:
		firmwareUpdateFileName_ = wxString(wxT("mvBlueLYNX-M7_Update.tgz"));
		productStringForFirmwareUpdateCheck_ = wxString(wxT("mvBlueLYNX-M7"));
		break;
	default:
		break;
	}

	static const wxString MVIMPACT_ACQUIRE_DIR(wxT("MVIMPACT_ACQUIRE_DIR"));
	if( !::wxGetEnv( MVIMPACT_ACQUIRE_DIR, &firmwareUpdateFolder_ ) )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxT("Can't install new firmware automatically as a crucial environment variable(MVIMPACT_ACQUIRE_DIR) is missing...\nPlease select the folder containing the files manually.\n") );
		}
	}
	else if( product_ == pgBlueFOX3 )
	{
		firmwareUpdateFolder_.append( wxT("/FirmwareUpdates/mvBlueFOX") );
	}
	else
	{
		firmwareUpdateFolder_.append( wxT("/FirmwareUpdates/mvBlueCOUGAR") );
	}
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::CheckForIncompatibleFirmwareVersions_BlueCOUGAR_X( bool boSilentMode, const wxString& serial, const FileEntryContainer& fileEntries, const wxString& selection, const Version& currentFirmwareVersion )
//-----------------------------------------------------------------------------
{
	// check for updates between incompatible firmware versions.
	const FileEntryContainer::size_type cnt = fileEntries.size();
	static const Version firstVersionWithNewNames(1, 1, 86, 0);
	bool boCompatibleInterfaces = true;
	FileEntryContainer::size_type fileIndex = 0;
	for( fileIndex=0; fileIndex<cnt; fileIndex++ )
	{
		if( fileEntries[fileIndex].name_ == selection )
		{
			if( ( currentFirmwareVersion < firstVersionWithNewNames ) &&
				( fileEntries[fileIndex].version_ >= firstVersionWithNewNames ) )
			{
				boCompatibleInterfaces = false;
				break;
			}
			else if( ( currentFirmwareVersion >= firstVersionWithNewNames ) &&
				( fileEntries[fileIndex].version_ < firstVersionWithNewNames ) )
			{
				boCompatibleInterfaces = false;
				break;
			}
		}
	}

	if( !boCompatibleInterfaces )
	{
		if( !MessageToUser( wxT("Firmware Update"), wxString::Format( wxT("There has been an incompatible interface change between the firmware version currently running on the device(%ld.%ld.%ld.%ld) and the version you are about to install(%ld.%ld.%ld.%ld).\nFor further information please refer to the mvBlueCOUGAR-X manual section 'C++ developer -> Porting existing code -> Notes about mvIMPACT Acquire version 1.11.32 or higher and/or using mvBlueCOUGAR-X with firmware 1.2.0 or higher'.\n\nAre you sure you want to continue updating the firmware of device %s?"),
			currentFirmwareVersion.major_, currentFirmwareVersion.minor_, currentFirmwareVersion.subMinor_, currentFirmwareVersion.release_,
			fileEntries[fileIndex].version_.major_, fileEntries[fileIndex].version_.minor_, fileEntries[fileIndex].version_.subMinor_, fileEntries[fileIndex].version_.release_,
			serial.c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), ConvertedString(serial).c_str() ) );
			}
			return urOperationCanceled;
		}
	}

	return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
DeviceHandler::TUpdateResult DeviceHandlerBlueDevice::DoFirmwareUpdate_BlueCOUGAR_X( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize )
//-----------------------------------------------------------------------------
{
	TUpdateResult result = urOperationSuccessful;
	{
		UpdateFeatures uf(pDev_);
		if( uf.transferBuffer_.isValid() )
		{
			uf.transferBuffer_.writeBinary( std::string(pBuf, bufSize) );
			int updateFirmwareCallResult = uf.updateFirmware_.call();
			if( updateFirmwareCallResult != DMR_NO_ERROR )
			{
				result = urDeviceAccessError;
			}
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Result of updating device %s: %s.\n"), ConvertedString(serial).c_str(), ConvertedString(uf.lastResult_.read()).c_str() ) );
			}
		}
		else
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Failed to update device %s(Feature 'TransferBuffer' is not implemented, a driver update is needed).\n"), ConvertedString(serial).c_str() ) );
			}
			result = urFeatureUnsupported;
		}
	}
	if( result == urOperationSuccessful )
	{
		// now check if the new firmware is actually active
		try
		{
			if( pDev_->isOpen() )
			{
				pDev_->close();
			}
			DeviceManager devMgr;
			devMgr.updateDeviceList(); // this is needed in order to invalidate cached data that might no longer be valid(e.g. the URLs for the description files)
			pDev_->open();
		}
		catch( const ImpactAcquireException& e )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Failed to close and re-open device %s(%s, %s).\n"), 
					ConvertedString(pDev_->serial.read()).c_str(),
					ConvertedString(e.getErrorString()).c_str(),
					ConvertedString(e.getErrorCodeAsString()).c_str() ) );
			}
		}

		PropertyI64 firmwareSource;
		DeviceComponentLocator locator(pDev_, dltInfo);
		locator.bindComponent( firmwareSource, "mvDeviceFirmwareSource" );
		if( !firmwareSource.isValid() )
		{
			// the name of this feature did change in FW version 1.2.0, thus when updating/downgrading to a version smaller than 1.2.0
			// we need to use to 'old' name.
			locator.bindComponent( firmwareSource, "FirmwareSource" );
		}
		if( firmwareSource.isValid() )
		{
			if( ConvertedString(firmwareSource.readS()) != wxString(wxT("ProgramSection")) )
			{
				MessageToUser( wxT("Warning"), wxT("The firmware update did not succeed. The device is running with a fallback firmware version now as the update section is damaged. Please repeat the firmware update and at the end power-cycle the device when this message shows up again. If afterwards the property 'Info/DeviceControl/FirmwareSource' does not show 'ProgramSection' please contact MATRIX VISION."), boSilentMode, wxOK | wxICON_INFORMATION );
			}
		}
		else
		{
			MessageToUser( wxT("Warning"), wxT("The firmware of this device could not be verified. Please Check if the firmware version displayed matches the expected version after power-cycling the device."), boSilentMode, wxOK | wxICON_INFORMATION );
		}
	}
	pDev_->close();
	return result;
}

//-----------------------------------------------------------------------------
DeviceHandler::TUpdateResult DeviceHandlerBlueDevice::DoFirmwareUpdate_BlueFOX3( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize )
//-----------------------------------------------------------------------------
{
	pDev_->interfaceLayout.write( dilGenICam );
	{
		// switch off automatic update of GenICam features triggered by recommended polling times in the GenICam XML file
		SystemSettingsGenTL ss(pDev_);
		ss.featurePollingEnable.write( bFalse );
	}

	mvIMPACT::acquire::GenICam::FileAccessControl fac(pDev_);
	int fileOperationExecuteResult = DMR_NO_ERROR;

	// erase the flash before downloading the new firmware
	fac.fileSelector.writeS( "DeviceFirmware" );
	fac.fileOperationSelector.writeS( "Delete" );
	fileOperationExecuteResult = fac.fileOperationExecute.call();
	if( static_cast<TDMR_ERROR>(fileOperationExecuteResult) != DMR_NO_ERROR )
	{
		MessageToUser( wxT("Warning"), wxString::Format( wxT("Failed to erase flash memory for firmware image on device %s. Error reported from driver: %d(%s)."), ConvertedString(serial).c_str(), fileOperationExecuteResult, ConvertedString(ImpactAcquireException::getErrorCodeAsString( fileOperationExecuteResult )).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
		return urFileIOError;
	}

	{
		// download the firmware image
		mvIMPACT::acquire::GenICam::ODevFileStream uploadFile;
		uploadFile.open( pDev_, "DeviceFirmware" );
		if( uploadFile.fail() )
		{
			MessageToUser( wxT("Warning"), wxString::Format( wxT("Failed to open firmware file on device %s."), ConvertedString(serial).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
			return urFileIOError;
		}

		uploadFile.write( pBuf, static_cast<std::streamsize>(bufSize) );
		if( uploadFile.fail() )
		{
			MessageToUser( wxT("Warning"), wxString::Format( wxT("Failed to upload firmware file to device %s."), ConvertedString(serial).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
			return urFileIOError;
		}
		uploadFile.close();
	}

	// activate the firmware
	fac.fileOperationSelector.writeS( "MvFlashWrite" );
	fileOperationExecuteResult = fac.fileOperationExecute.call();
	if( static_cast<TDMR_ERROR>(fileOperationExecuteResult) != DMR_NO_ERROR )
	{
		MessageToUser( wxT("Warning"), wxString::Format( wxT("Activating the firmware downloaded to device %s failed. Please re-try to update the firmware and do NOT yet power-cycle the device. Error reported from driver: %d(%s)."), ConvertedString(serial).c_str(), fileOperationExecuteResult, ConvertedString(ImpactAcquireException::getErrorCodeAsString( fileOperationExecuteResult )).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
		return urFileIOError;
	}


	// download the file again to check its integrity
	auto_array_ptr<char> pBufCheck(bufSize);
	memset( pBufCheck.get(), 0, bufSize );
	{
		mvIMPACT::acquire::GenICam::IDevFileStream downloadFile;
		downloadFile.open( pDev_, "DeviceFirmware" );
		if( downloadFile.fail() )
		{
			MessageToUser( wxT("Warning"), wxString::Format( wxT("Failed to open firmware file on device %s while trying to verify the firmware."), ConvertedString(serial).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
			return urFileIOError;
		}

		downloadFile.read( pBufCheck, static_cast<std::streamsize>(bufSize) );
		if( downloadFile.fail() )
		{
			MessageToUser( wxT("Warning"), wxString::Format( wxT("Failed to download firmware file from device %s."), ConvertedString(serial).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
			return urFileIOError;
		}
		downloadFile.close();
	}

	if( memcmp( pBuf, pBufCheck, bufSize ) != 0 )
	{
		MessageToUser( wxT("Warning"), wxString::Format( wxT("The verification of the firmware downloaded to device %s failed. Please re-try to update the firmware and do NOT yet power-cycle the device."), ConvertedString(serial).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("The verification of the firmware downloaded to device %s failed. Please re-try to update the firmware and do NOT yet power-cycle the device. A detailed list of diffenrences will follow now."), ConvertedString(serial).c_str() ) );
			for( size_t i=0; i<bufSize; i++ )
			{
				if( pBuf[i] != pBufCheck[i] )
				{
					pParent_->WriteLogMessage( wxString::Format( wxT( "Byte[%08u(0x%08x]: Expected: 0x%02x, got: 0x%02x\n"), i, i, pBuf[i], pBufCheck[i] ) );
				}
			}
		}
		return urFileIOError;
	}

	mvIMPACT::acquire::GenICam::DeviceControl dc(pDev_);
	if( dc.deviceReset.isValid() )
	{
		const int deviceResetResult = dc.deviceReset.call();
		MessageToUser( wxT("Update Result"), wxString::Format( wxT("Update successful but resetting device failed. Please disconnect and reconnect device %s now to activate the new firmware. Error reported from driver: %d(%s)."), ConvertedString(serial).c_str(), deviceResetResult, ConvertedString(ImpactAcquireException::getErrorCodeAsString( deviceResetResult )).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
	}
	else
	{
		MessageToUser( wxT("Update Result"), wxString::Format( wxT("Update successful. Please disconnect and reconnect device %s now to activate the new firmware."), ConvertedString(serial).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
	}

	pDev_->close();
	wxMilliSleep( 10000 );

	return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::ExtractFileVersion( const wxString& fileName, Version& fileVersion ) const
//-----------------------------------------------------------------------------
{
	static const wxString TOKEN(wxT("_GigE_"));
	int pos = fileName.Find( TOKEN.c_str() );
	if( pos == wxNOT_FOUND )
	{
		return false;
	}
	wxString versionString(fileName.Mid( pos + TOKEN.Length() ));
	return ( GetNextNumber( versionString, fileVersion.major_ ) &&
		GetNextNumber( versionString, fileVersion.minor_ ) &&
		GetNextNumber( versionString, fileVersion.subMinor_ ) );
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::GetFileFromArchive( const wxString& firmwareFileAndPath, const char* pArchive, size_t archiveSize, const wxString& filename, auto_array_ptr<char>& data )
//-----------------------------------------------------------------------------
{
	wxMemoryInputStream zipData(pArchive, archiveSize);
	wxZipInputStream zipStream(zipData);
	wxZipEntry* pZipEntry = 0; 
	while( ( pZipEntry = zipStream.GetNextEntry() ) != 0 )
	{
		/// GetNextEntry returns the ownership to the object!
		std::auto_ptr<wxZipEntry> ptrGuard(pZipEntry);
		if( !zipStream.OpenEntry( *pZipEntry ) )
		{
			if( pParent_ )
			{
				pParent_->WriteErrorMessage( wxString::Format( wxT("Failed to open zip entry of archive %s\n"), firmwareFileAndPath.c_str() ) );
			}
			continue;
		}

		if( pZipEntry->IsDir() )
		{
			continue;
		}

		int pos = pZipEntry->GetInternalName().Find( filename.c_str() );
		if( pos == wxNOT_FOUND )
		{
			continue;
		}

		if( pZipEntry->GetInternalName().Mid( pos ) == filename )
		{
			data.realloc( pZipEntry->GetSize() );
			zipStream.Read( data.get(), data.parCnt() );
			return true;
		}
	}
	return false;
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::GetIDFromUser( long& /*newID*/, const long /*minValue*/, const long /*maxValue*/ )
//-----------------------------------------------------------------------------
{
	ConvertedString serial(pDev_->serial.read());
	wxString tool;
	wxString standard;
	switch( product_ )
	{
	case pgUnknown:
		break;
	case pgBlueFOX3:
		tool = wxT("wxPropView");
		standard = wxT("USB3 Vision");
		break;
	case pgBlueCOUGAR_P:
	case pgBlueCOUGAR_S:
	case pgBlueCOUGAR_X:
	case pgBlueLYNX_M7:
		tool = wxT("mvIPConfigure");
		standard = wxT("GigE Vision");
		break;
	}

	if( tool.IsEmpty() || standard.IsEmpty() )
	{
		if( MessageToUser( wxT("Set Device ID"), wxString::Format( wxT("No valid tool/standard could be located for setting the device ID of device %s"), serial.c_str() ), false, wxOK | wxICON_ERROR ) )
		return false;
	}

	if( MessageToUser( wxT("Set Device ID"), wxString::Format( wxT("Device %s does not support setting an integer device ID.\n\nAs this device is %s compliant it might however support setting a user defined string by modifying the property 'DeviceUserID' that can be used instead.\n\nThis string can be assigned using the tool '%s'. Do you want to launch this application now?"), serial.c_str(), standard.c_str(), tool.c_str() ), false, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
	{
		::wxExecute( tool );
	}
	return false;
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::IsBlueCOUGAR_X( mvIMPACT::acquire::Device* pDev )
//-----------------------------------------------------------------------------
{
	if( ConvertedString(pDev->product.read()).Find( wxT("mvBlueCOUGAR-X") ) != wxNOT_FOUND )
	{
		return true;
	}

	ComponentLocator locator(pDev->hDev());
	Property prop;
	locator.bindComponent( prop, "DeviceMACAddress" );
	if( !prop.isValid() )
	{
		return false;
	}

	int64_type MAC = MACAddressFromString( prop.readS() );
	if( ( ( MAC >= 0x0000000c8d600001LL ) && ( MAC <= 0x0000000c8d60ffffLL ) ) ||
		( ( MAC >= 0x0000000c8d610001LL ) && ( MAC <= 0x0000000c8d61ffffLL ) ) ||
		( ( MAC >= 0x0000000c8d700001LL ) && ( MAC <= 0x0000000c8d707fffLL ) ) ||
		( ( MAC >= 0x0000000c8d708001LL ) && ( MAC <= 0x0000000c8d70bfffLL ) ) ||
		( ( MAC >= 0x0000000c8d70c001LL ) && ( MAC <= 0x0000000c8d70cfffLL ) ) )
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::IsBlueFOX3( mvIMPACT::acquire::Device* pDev )
//-----------------------------------------------------------------------------
{
	if( ConvertedString(pDev->product.read()).Find( wxT("mvBlueFOX3") ) != wxNOT_FOUND )
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------
void DeviceHandlerBlueDevice::SetCustomFirmwarePath( const wxString& customFirmwarePath )
//-----------------------------------------------------------------------------
{
	if( !customFirmwarePath.IsEmpty() )
	{
		firmwareUpdateFolder_ = customFirmwarePath;
	}
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::SupportsFirmwareUpdate( void ) const
//-----------------------------------------------------------------------------
{
	return !firmwareUpdateFileName_.IsEmpty();
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateCOUGAR_SDevice( bool boSilentMode )
//-----------------------------------------------------------------------------
{
	ConvertedString serial(pDev_->serial.read());
	wxString firmwareFileAndPath;
	wxString descriptionFileAndPath;
	bool boGotDescriptionFile = false;
	if( boSilentMode )
	{
		firmwareFileAndPath = firmwareUpdateFolder_ + wxString(wxT("/")) + firmwareUpdateFileName_;
		wxArrayString descriptionFiles;
		wxDir::GetAllFiles( firmwareUpdateFolder_, &descriptionFiles, wxT("MATRIXVISION_mvBlueCOUGAR-S_GigE_*.zip") );
		const size_t cnt = descriptionFiles.GetCount();
		Version highestVersion;
		for( size_t i=0; i<cnt; i++ )
		{
			Version currentVersion;
			wxFileName fileName(descriptionFiles[i]);
			if( ExtractFileVersion( fileName.GetName(), currentVersion ) )
			{
				if( highestVersion < currentVersion )
				{
					descriptionFileAndPath = descriptionFiles[i];
					boGotDescriptionFile = true;
					highestVersion = currentVersion;
				}
			}
			else if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Failed to parse filenames major version number(%s).\n"), descriptionFiles[i].c_str() ) );
			}
		}
	}

	if( !boGotDescriptionFile )
	{
		wxFileDialog fileDlg(pParent_, wxT("Please select the firmware file"), firmwareUpdateFolder_, firmwareUpdateFileName_, wxString::Format( wxT("%s firmware file (%s)|%s"), ConvertedString(pDev_->product.read()).c_str(), firmwareUpdateFileName_.c_str(), firmwareUpdateFileName_.c_str() ), wxOPEN | wxFILE_MUST_EXIST);
		if( fileDlg.ShowModal() != wxID_OK )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), ConvertedString(serial).c_str() ) );
			}
			return urOperationCanceled;
		}

		if( fileDlg.GetFilename().Find( productStringForFirmwareUpdateCheck_.c_str() ) != 0 )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("The file %s is not meant for updating device %s.\n"), fileDlg.GetFilename().c_str(), serial.c_str() ), wxTextAttr(wxColour(255, 0, 0)) );
			}
			return urInvalidFileSelection;
		}

		firmwareFileAndPath = fileDlg.GetPath();

		wxFileDialog fileDlgDescriptionFile(pParent_, wxT("Please select the GenICam description file that came with the firmware"), firmwareUpdateFolder_, wxT(""), wxString::Format( wxT("%s GenICam description file (MATRIXVISION_mvBlueCOUGAR-S_GigE*.zip)|MATRIXVISION_mvBlueCOUGAR-S_GigE*.zip"), ConvertedString(pDev_->product.read()).c_str() ), wxOPEN | wxFILE_MUST_EXIST);
		if( fileDlgDescriptionFile.ShowModal() != wxID_OK )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), ConvertedString(serial).c_str() ) );
			}
			return urOperationCanceled;
		}

		if( fileDlgDescriptionFile.GetFilename().Find( wxT("MATRIXVISION_mvBlueCOUGAR-S_GigE") ) != 0 )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("The file %s is not meant for updating device %s.\n"), fileDlgDescriptionFile.GetFilename().c_str(), serial.c_str() ), wxTextAttr(wxColour(255, 0, 0)) );
			}
			return urInvalidFileSelection;
		}

		descriptionFileAndPath = fileDlgDescriptionFile.GetPath();
	}

	if( !MessageToUser( wxT("Firmware Update"), wxString::Format( wxT("Are you sure you want to update the firmware and description file of device %s?\nThis might take several minutes during which the application will not react.\n\nPlease be patient."), serial.c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), ConvertedString(serial).c_str() ) );
		}
		return urOperationCanceled;
	}

	int result = UploadFile( descriptionFileAndPath, descriptionFileAndPath );
	if( result != urOperationSuccessful )
	{
		return result;
	}
	result = UploadFile( firmwareFileAndPath, descriptionFileAndPath );
	if( result == urOperationSuccessful )
	{
		MessageToUser( wxT("Information"), wxT("The Firmware has been updated. The new Firmware is active now and the device can be used again by other applications."), boSilentMode, wxOK | wxICON_INFORMATION );
	}
	return result;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateCOUGAR_XAndFOX3Device( bool boSilentMode )
//-----------------------------------------------------------------------------
{
	ConvertedString serial(pDev_->serial.read());
	wxString firmwareFileAndPath;
	wxString descriptionFileAndPath;

	if( boSilentMode )
	{
		firmwareFileAndPath = firmwareUpdateFolder_ + wxString(wxT("/")) + firmwareUpdateFileName_;
	}
	else
	{
		wxFileDialog fileDlg(pParent_, wxT("Please select the update archive containing the firmware"), firmwareUpdateFolder_, firmwareUpdateFileName_, wxString::Format( wxT("%s update archive (%s)|%s"), ConvertedString(pDev_->product.read()).c_str(), firmwareUpdateFileName_.c_str(), firmwareUpdateFileName_.c_str() ), wxOPEN | wxFILE_MUST_EXIST);
		if( fileDlg.ShowModal() != wxID_OK )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), serial.c_str() ) );
			}
			return urOperationCanceled;
		}
		firmwareFileAndPath = fileDlg.GetPath();
	}

	wxFFile archive(firmwareFileAndPath.c_str(), wxT("rb"));
	if( !archive.IsOpened() )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("Could not open update archive %s.\n"), firmwareFileAndPath.c_str() ) );
		}
		return urFileIOError;
	}

	auto_array_ptr<char> pBuffer(archive.Length());
	size_t bytesRead = archive.Read( pBuffer.get(), pBuffer.parCnt() );
	if( bytesRead != static_cast<size_t>(archive.Length()) )
	{
		if( pParent_ )
		{
			wxString lengthS;
			lengthS << archive.Length();
			pParent_->WriteErrorMessage( wxString::Format( wxT("Could not read full content of archive '%s'(wanted: %s, bytes, got %d bytes).\n"), firmwareFileAndPath.c_str(), lengthS.c_str(), bytesRead ) );
		}
		return urFileIOError;
	}

	wxMemoryInputStream zipData(pBuffer.get(), pBuffer.parCnt());
	wxZipInputStream zipStream(zipData);
	int fileCount = zipStream.GetTotalEntries();
	if( fileCount < 2 )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("The archive %s claims to contain %d file(s) while at least 2 are needed\n"), firmwareFileAndPath.c_str(), fileCount) );
		}
		return urInvalidFileSelection;
	}

	auto_array_ptr<char> pPackageDescription;
	if( !GetFileFromArchive( firmwareFileAndPath, pBuffer.get(), pBuffer.parCnt(), wxString(wxT("packageDescription.xml")), pPackageDescription ) )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("Could not extract package description from archive %s.\n"), firmwareFileAndPath.c_str() ) );
		}
		return urFileIOError;
	}

	PackageDescriptionFileParser fileParser;
	fileParser.Create();
	bool fSuccess = true;
	char* pszBuffer = static_cast<char*>(fileParser.GetBuffer( static_cast<int>(pPackageDescription.parCnt()) + 1 ));
	if( !pszBuffer )
	{
		fSuccess = false;
	}
	else
	{
		memcpy( pszBuffer, pPackageDescription.get(), pPackageDescription.parCnt() );
		pszBuffer[pPackageDescription.parCnt()] = '\0';
		fSuccess = fileParser.ParseBuffer( static_cast<int>(pPackageDescription.parCnt()), true );
	}

	if( !fSuccess || ( fileParser.GetErrorCode() != XML_ERROR_NONE ) )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("Package description file parser error(fSuccess: %d, XML error: %d(%s)).\n"), fSuccess, fileParser.GetErrorCode(), fileParser.GetErrorString(fileParser.GetErrorCode()) ) );
		}
		return urFileIOError;
	}

	if( !fileParser.GetLastError().empty() )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("Package description file parser error(last error: %s).\n"), fileParser.GetLastError().c_str() ) );
		}
		return urFileIOError;
	}

	const FileEntryContainer& fileEntries = fileParser.GetResults();
	const FileEntryContainer::size_type cnt = fileEntries.size();
	wxString product(ConvertedString(pDev_->product.read()));
	wxString deviceVersionAsString(pDev_->deviceVersion.isValid() ? ConvertedString(pDev_->deviceVersion.read()) : wxString(wxT("1.0")));
	Version deviceVersion(VersionFromString( deviceVersionAsString ));
	wxArrayString choices;
	wxArrayString unusableFiles;
	for( FileEntryContainer::size_type i=0; i<cnt; i++ )
	{
		std::map<wxString, SuitableProductKey>::const_iterator it = fileEntries[i].suitableProductKeys_.find( product );
		bool boSuitable = it != fileEntries[i].suitableProductKeys_.end();
		if( boSuitable && ( fileEntries[i].type_ == wxString(wxT("Firmware")) ) &&
			IsVersionWithinRange( deviceVersion, it->second.revisionMin_, it->second.revisionMax_ ) )
		{
			choices.Add( wxString::Format( wxT("%s(%s, file version: %ld.%ld.%ld.%ld)"), fileEntries[i].name_.c_str(), fileEntries[i].description_.c_str(), fileEntries[i].version_.major_, fileEntries[i].version_.minor_, fileEntries[i].version_.subMinor_, fileEntries[i].version_.release_ ) );
		}
		else
		{
			wxString suitableProducts;
			it = fileEntries[i].suitableProductKeys_.begin();
			while( it != fileEntries[i].suitableProductKeys_.end() )
			{
				suitableProducts.Append( wxString::Format( wxT("  %s(%s, rev. %ld.%ld.%ld - %ld.%ld.%ld)\n"), it->first.c_str(), it->second.name_.c_str(),
					it->second.revisionMin_.major_, it->second.revisionMin_.minor_, it->second.revisionMin_.release_,
					it->second.revisionMax_.major_, it->second.revisionMax_.minor_, it->second.revisionMax_.release_ ) );
				++it;
			}
			unusableFiles.Add( wxString::Format( wxT("%s(%s, file version: %ld.%ld.%ld.%ld), suitable products:\n %s"), fileEntries[i].name_.c_str(), fileEntries[i].description_.c_str(), fileEntries[i].version_.major_, fileEntries[i].version_.minor_, fileEntries[i].version_.subMinor_, fileEntries[i].version_.release_, suitableProducts.c_str() ) );
		}
	}
	choices.Sort( CompareFileVersion );

	wxString selection;
	if( choices.IsEmpty() )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("The archive %s does not contain a suitable firmware file for device %s(%s, Rev. %s). Files detected:\n"), firmwareFileAndPath.c_str(), serial.c_str(), product.c_str(), deviceVersionAsString.c_str() ) );
			wxArrayString::size_type cnt = unusableFiles.size();
			for( wxArrayString::size_type i=0; i<cnt; i++ )
			{
				pParent_->WriteErrorMessage( unusableFiles[i] );
			}

		}
		return urInvalidFileSelection;
	}
	else if( ( choices.size() == 1 ) || boSilentMode )
	{
		selection = choices[0].BeforeFirst( wxT('(') );
	}
	else
	{
		wxSingleChoiceDialog dlg(pParent_, wxT("This archive contains more than one firmware file.\nPlease select a file you want to program the device with."), wxT("Please select a file to upload and install on the device"), choices);
		if( dlg.ShowModal() == wxID_OK )
		{
			selection = dlg.GetStringSelection().BeforeFirst( wxT('(') );
		}
		else
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), serial.c_str() ) );
			}
			return urOperationCanceled;
		}
	}

	const Version currentFirmwareVersion = VersionFromString( ConvertedString(pDev_->firmwareVersion.readS()) );
	if( product_ == pgBlueCOUGAR_X )
	{
		const int incompatibleFirmwareCheckResult = CheckForIncompatibleFirmwareVersions_BlueCOUGAR_X( boSilentMode, serial, fileEntries, selection, currentFirmwareVersion );
		if( incompatibleFirmwareCheckResult != urOperationSuccessful )
		{
			return incompatibleFirmwareCheckResult;
		}
	}

	if( boSilentMode )
	{
		for( FileEntryContainer::size_type i=0; i<cnt; i++ )
		{
			if( fileEntries[i].name_ == selection )
			{
				if( currentFirmwareVersion >= fileEntries[i].version_ )
				{
					if( pParent_ )
					{
						pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s as the current firmware version(%ld.%ld.%ld.%ld) is newer or equal as the one provided in the archive(%ld.%ld.%ld.%ld).\n"),
							ConvertedString(serial).c_str(),
							currentFirmwareVersion.major_, currentFirmwareVersion.minor_, currentFirmwareVersion.subMinor_, currentFirmwareVersion.release_,
							fileEntries[i].version_.major_, fileEntries[i].version_.minor_, fileEntries[i].version_.subMinor_, fileEntries[i].version_.release_ ) );
					}
					return urOperationCanceled;
				}
				else
				{
					// different version -> proceed
					break;
				}
			}
		}
	}

	auto_array_ptr<char> pUploadFileBuffer;
	if( !GetFileFromArchive( firmwareFileAndPath, pBuffer.get(), pBuffer.parCnt(), selection, pUploadFileBuffer ) )
	{
		if( pParent_ )
		{
			pParent_->WriteErrorMessage( wxString::Format( wxT("Could not extract file %s from archive %s.\n"), selection.c_str(), firmwareFileAndPath.c_str() ) );
		}
		return urFileIOError;
	}

	if( !MessageToUser( wxT("Firmware Update"), wxString::Format( wxT("Are you sure you want to update the firmware of device %s?\nThis might take several minutes during which the application will not react.\n\nPlease be patient."), serial.c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), ConvertedString(serial).c_str() ) );
		}
		return urOperationCanceled;
	}


	TUpdateResult result = urOperationSuccessful;
	try
	{
		switch( product_ )
		{
		case pgBlueCOUGAR_X:
			result = DoFirmwareUpdate_BlueCOUGAR_X( boSilentMode, serial, pUploadFileBuffer.get(), pUploadFileBuffer.parCnt() );
			break;
		case pgBlueFOX3:
			result = DoFirmwareUpdate_BlueFOX3( boSilentMode, serial, pUploadFileBuffer.get(), pUploadFileBuffer.parCnt() );
			break;
		default:
			result = urFeatureUnsupported;
			break;
		}
	}
	catch( const ImpactAcquireException& e )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Failed to update device %s(%s, %s).\n"), 
				ConvertedString(serial).c_str(),
				ConvertedString(e.getErrorString()).c_str(),
				ConvertedString(e.getErrorCodeAsString()).c_str() ) );
		}
		if( pDev_->isOpen() )
		{
			pDev_->close();
		}
		result = urDeviceAccessError;
	}

	if( result == urOperationSuccessful )
	{
		MessageToUser( wxT("Information"), wxT("The Firmware has been updated. The new Firmware is active now and the device can be used again by other applications."), boSilentMode, wxOK | wxICON_INFORMATION );
	}
	return result;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateFirmware( bool boSilentMode )
//-----------------------------------------------------------------------------
{
	ConvertedString serial(pDev_->serial.read());
	if( !MessageToUser( wxT("Firmware Update"), wxString::Format( wxT("Please close all other applications using device %s before proceeding."), serial.c_str() ), boSilentMode, wxCANCEL | wxOK | wxICON_INFORMATION ) )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), serial.c_str() ) );
		}
		return urOperationCanceled;
	}

	if( product_ == pgBlueCOUGAR_S )
	{
		return UpdateCOUGAR_SDevice( boSilentMode );
	}
	else if( ( product_ == pgBlueCOUGAR_X ) ||
			( product_ == pgBlueFOX3 ) )
	{
		return UpdateCOUGAR_XAndFOX3Device( boSilentMode );
	}
	else
	{
		return UpdateLYNX_M7AndCOUGAR_PDevice( firmwareUpdateFileName_, wxString(wxT(".tgz")), boSilentMode );
	}
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateLYNX_M7AndCOUGAR_PDevice( const wxString& updateFileName, const wxString& /* fileExtension */, bool boSilentMode )
//-----------------------------------------------------------------------------
{
	ConvertedString serial(pDev_->serial.read());
	wxFileDialog fileDlg(pParent_, wxT("Please select the firmware file"), firmwareUpdateFolder_, updateFileName, wxString::Format( wxT("%s update file (%s)|%s"), ConvertedString(pDev_->product.read()).c_str(), updateFileName.c_str(), updateFileName.c_str() ), wxOPEN | wxFILE_MUST_EXIST);
	if( fileDlg.ShowModal() != wxID_OK )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Firmware update canceled for device %s.\n"), serial.c_str() ) );
		}
		return urOperationCanceled;
	}

	if( fileDlg.GetFilename().Find( productStringForFirmwareUpdateCheck_.c_str() ) != 0 )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("The file %s is not meant for updating device %s.\n"), fileDlg.GetFilename().c_str(), serial.c_str() ), wxTextAttr(wxColour(255, 0, 0)) );
		}
		return urInvalidFileSelection;
	}

	if( !MessageToUser( wxT("Firmware Update"), wxString::Format( wxT("Are you sure you want to update the firmware of device %s?\nThis might take several minutes during which the application will not react.\n\nPlease be patient.\nWARNING: During the firmware update all data stored on the device by a user\nlike applications or installed ipks will be deleted!"), serial.c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("firmware update canceled for device %s.\n"), serial.c_str() ) );
		}
		return urOperationCanceled;
	}

	try
	{
		pDev_->open();
	}
	catch( const ImpactAcquireException& e )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Failed to open device %s(%s, %s).\n"),
				serial.c_str(),
				ConvertedString(e.getErrorString()).c_str(),
				ConvertedString(e.getErrorCodeAsString()).c_str() ) );
		}
		return urDeviceAccessError;
	}

	try
	{
		UpdateFeatures uf(pDev_);
		uf.transferMode_.writeS( "Binary" );

		wxString directory(fileDlg.GetDirectory());
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxT("Installing updates. This will take some minutes. During this time the application will not react.\n") );
		}

		// upload the file
		uf.sourceFileName_.write( std::string(wxConvCurrent->cWX2MB(fileDlg.GetPath().c_str())) );
		uf.destinationFileName_.write( std::string(wxConvCurrent->cWX2MB(fileDlg.GetFilename().c_str())) );
		int functionResult = DMR_NO_ERROR;
		if( ( functionResult = uf.upload_.call( std::string(wxConvCurrent->cWX2MB( wxT("") )) ) ) == DMR_NO_ERROR )
		{
			bool boUpdateDone = false;
			bool boNewUpdateFeatureUsed = false;
			// install the file
			if( uf.updateFirmware_.isValid() ) // check if the new method is available
			{
				if( ( functionResult = uf.updateFirmware_.call( std::string(wxConvCurrent->cWX2MB( wxT("") )) ) ) == DMR_NO_ERROR )
				{
					boUpdateDone = true;
					boNewUpdateFeatureUsed = true;
				}
				else
				{
					if( pParent_ )
					{
						pParent_->WriteLogMessage( wxString::Format( wxT("Failed to install %s using the new firmware update procedure: %d(%s) %s.\n"), 
							fileDlg.GetFilename().c_str(),
							functionResult,
							ConvertedString(ImpactAcquireException::getErrorCodeAsString( functionResult )).c_str(),
							ConvertedString(uf.lastResult_.read()).c_str() ) );
					}
				}
			}

			if( !boUpdateDone )
			{
				if( pParent_ )
				{
					pParent_->WriteLogMessage( wxT("Trying deprecated firmware update procedure now.\n") );
				}
				if( ( functionResult = uf.install_.call( std::string(wxConvCurrent->cWX2MB( wxT("") )) ) ) == DMR_NO_ERROR )
				{
					boUpdateDone = true;
				}
				else if( pParent_ )
				{
					pParent_->WriteLogMessage( wxString::Format( wxT("Failed to install %s: %d(%s) %s\n"), 
						fileDlg.GetFilename().c_str(),
						functionResult,
						ConvertedString(ImpactAcquireException::getErrorCodeAsString( functionResult )).c_str(),
						ConvertedString(uf.lastResult_.read()).c_str() ) );
				}
			}
			if( boUpdateDone )
			{
				if( pParent_ )
				{
					pParent_->WriteLogMessage( wxString::Format( wxT("Result of the installation of %s: %s\n"), fileDlg.GetPath().c_str(), ConvertedString(uf.lastResult_.read()).c_str() ) );
				}
				// download the installation log file
				wxString logFileName;
				wxString fullLogFilePath;
				if( boNewUpdateFeatureUsed )
				{
					// during the new method the device will reboot, thus we must re-establish the connection to it.
					pDev_->close();
					pDev_->open();
					uf.bindFeatures();
					DeviceComponentLocator locator(pDev_, dltInfo);
					PropertyS firmwareLogFilePathProp(locator.findComponent( "DeviceFirmwareUpdateLogFilePath" ));
					if( firmwareLogFilePathProp.isValid() )
					{
						fullLogFilePath = ConvertedString(firmwareLogFilePathProp.read());
						size_t pos = fullLogFilePath.find_last_of( wxT("/") );
						if( pos != wxString::npos )
						{
							logFileName = fullLogFilePath.Mid( pos + 1 );
						}
						else if( pParent_ )
						{
							pParent_->WriteLogMessage( wxString::Format( wxT("Can't obtain log file name from path %s\n"), fullLogFilePath.c_str() ) );
						}
					}
					else if( pParent_ )
					{
						pParent_->WriteLogMessage( wxT("Can't obtain log file path\n") );
					}
				}
				else
				{
					logFileName = fileDlg.GetFilename();
					logFileName.append( wxT(".install.log") );
					fullLogFilePath = wxT("/tmp/");
					fullLogFilePath.append( logFileName );
				}
				uf.sourceFileName_.write( std::string(wxConvCurrent->cWX2MB(fullLogFilePath.c_str())) );
				wxString logFileDestinationName(directory);
				logFileDestinationName.append( wxT("/") );
				logFileDestinationName.append( logFileName );
				uf.destinationFileName_.write( std::string(wxConvCurrent->cWX2MB(logFileDestinationName.c_str())) );
				uf.download_.call( std::string(wxConvCurrent->cWX2MB( wxT("") )) );
				if( pParent_ )
				{
					pParent_->WriteLogMessage( wxString::Format( wxT("Result of the downloading the installation log file %s to %s: %s\n"), fileDlg.GetFilename().c_str(), logFileDestinationName.c_str(), ConvertedString(uf.lastResult_.read()).c_str() ) );
				}

				// try to display and remove the *.log-file
				{
					wxFFile logFile(logFileDestinationName.c_str(), wxT("r"));
					if( logFile.IsOpened() )
					{
						wxString logData;
						logFile.ReadAll( &logData );
						if( pParent_ )
						{
							pParent_->WriteLogMessage( logData );
						}
					}
					else if( pParent_ )
					{
						pParent_->WriteLogMessage( wxString::Format( wxT("ERROR! Couldn't open temporary file %s for reading.\n"), logFileDestinationName.c_str() ), wxTextAttr(wxColour(255, 0, 0)) );
					}
				}

				if( !::wxRemoveFile( logFileDestinationName ) && pParent_ )
				{
					pParent_->WriteLogMessage( wxString::Format( wxT("ERROR! Couldn't delete temporary file %s. Must be deleted manually.\n"), logFileDestinationName.c_str() ), wxTextAttr(wxColour(255, 0, 0)) );
				}

				MessageToUser( wxT("Information"), wxT("The firmware has been updated. Please close this and all other applications\nusing this device and disconnect the device from the power supply.\nAfter rebooting the new firmware will be active.\n"), boSilentMode, wxOK | wxICON_INFORMATION );
			}
		}
		else if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Failed to upload %s: %d(%s) %s\n"), 
				fileDlg.GetFilename().c_str(),
				functionResult,
				ConvertedString(ImpactAcquireException::getErrorCodeAsString( functionResult )).c_str(),
				ConvertedString(uf.lastResult_.read()).c_str() ) );
		}
	}
	catch( const ImpactAcquireException& e )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Failed to update device %s(%s, %s).\n"),
				serial.c_str(),
				ConvertedString(e.getErrorString()).c_str(),
				ConvertedString(e.getErrorCodeAsString()).c_str() ) );
		}
		pDev_->close();
		return urDeviceAccessError;
	}
	pDev_->close();
	return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UploadFile( const wxString& fullPath, const wxString& descriptionFile )
//-----------------------------------------------------------------------------
{
	int result = urOperationSuccessful;
	ConvertedString serial(pDev_->serial.read());
	try
	{
		ComponentLocator locatorDevice(pDev_->hDev());
		PropertyI descriptionToUse(locatorDevice.findComponent( "DescriptionToUse" ));
		descriptionToUse.writeS( "CustomFile" );
		PropertyS customDescriptionFilename(locatorDevice.findComponent( "CustomDescriptionFileName" ));
		customDescriptionFilename.write( std::string(wxConvCurrent->cWX2MB(descriptionFile.c_str())) );
		pDev_->open();
	}
	catch( const ImpactAcquireException& e )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Failed to open device %s(%s, %s).\n"),
				serial.c_str(),
				ConvertedString(e.getErrorString()).c_str(),
				ConvertedString(e.getErrorCodeAsString()).c_str() ) );
		}
		return urDeviceAccessError;
	}

	try
	{
		UpdateFeatures uf(pDev_);
		uf.transferMode_.writeS( "Binary" );
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Installing %s. This will take some minutes. During this time the application will not react.\n"), fullPath.c_str() ) );
		}

		// upload the file
		uf.sourceFileName_.write( std::string(wxConvCurrent->cWX2MB(fullPath.c_str())) );
		uf.upload_.call( std::string(wxConvCurrent->cWX2MB( wxT("") )) );
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Result of the installation of %s: %s\n"), fullPath.c_str(), ConvertedString(uf.lastResult_.read()).c_str() ) );
		}
	}
	catch( const ImpactAcquireException& e )
	{
		if( pParent_ )
		{
			pParent_->WriteLogMessage( wxString::Format( wxT("Failed to update device %s(%s, %s).\n"),
				serial.c_str(),
				ConvertedString(e.getErrorString()).c_str(),
				ConvertedString(e.getErrorCodeAsString()).c_str() ) );
		}
		result = urDeviceAccessError;
	}
	pDev_->close();
	return result;
}
