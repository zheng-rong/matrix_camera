//-----------------------------------------------------------------------------
#include "DeviceConfigureFrame.h"
#include "DeviceHandlerHYPERION.h"
#include "KernelDriverUpdate.h"
#include "Epcs.h"
#include <sstream>
#include "wx/msgdlg.h"
#include "wx/ffile.h"
#include "wx/numdlg.h"
#include <apps/Common/wxAbstraction.h>

using namespace std;

//-----------------------------------------------------------------------------
bool DeviceHandlerHYPERION::SupportsDMABufferSizeUpdate( int* pCurrentDMASize_kB /* = 0 */ )
//-----------------------------------------------------------------------------
{
	if( pCurrentDMASize_kB )
	{
		try
		{
			ComponentLocator locator(pDev_->hDev());
			Method readDMABufferSize(locator.findComponent( "ReadPermanentDMABufferSize@i" ));
			*pCurrentDMASize_kB = readDMABufferSize.call() / 1024;
		}
		catch( const ImpactAcquireException& e )
		{
			if( pParent_ )
			{
				pParent_->WriteErrorMessage( wxString::Format( wxT("Failed to query DMA memory size for %s(%s, %s).\n"), 
					ConvertedString(pDev_->serial.read()).c_str(),
					ConvertedString(e.getErrorString()).c_str(),
					ConvertedString(e.getErrorCodeAsString()).c_str() ) );
			}
		}
	}
	return true;
}

//-----------------------------------------------------------------------------
int DeviceHandlerHYPERION::UpdatePermanentDMABufferSize( bool /*boSilentMode*/ )
//-----------------------------------------------------------------------------
{
	int result = urOperationSuccessful;
	const long MB = (1024*1024);
	string devSerial = pDev_->serial.read();
	if( pParent_ )
	{
		try
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxT("Trying to update permanent DMA buffer size now. Please note that this change will not become effective until the system has been rebooted.\n" ) );
			}
			ComponentLocator locator(pDev_->hDev());
			Method readDMABufferSize(locator.findComponent( "ReadPermanentDMABufferSize@i" ));
			Method writeDMABufferSize(locator.findComponent( "WritePermanentDMABufferSize@ii" ));

			long newSize_Megabytes = wxGetNumberFromUser( wxT("Enter the new size for the permanent DMA buffer.\nPlease note that this change will not become effective until the system has been rebooted.\n"),
				wxT("New Size in Megabytes:"),
				wxString::Format( wxT("Set new DMA buffer size for %s(%s)"), ConvertedString(devSerial).c_str(), ConvertedString(pDev_->family.read()).c_str() ),
				(readDMABufferSize.call() / MB), // start
				0, // min
				0x7FFFFFFF, // max
				pParent_ );
			if( newSize_Megabytes >= 0 )
			{
				std::stringstream sstr;
				sstr.str("");
				sstr << (newSize_Megabytes*MB) << endl;
				writeDMABufferSize.call(sstr.str());
			}
			else
			{
				result = -4;
				pParent_->WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
			}
		}
		catch( const ImpactAcquireException& e )
		{
			if( pParent_ )
			{
				pParent_->WriteErrorMessage( wxString::Format( wxT("Failed to update DMA memory: %s(%s, %s).\n"), 
					ConvertedString(devSerial).c_str(),
					ConvertedString(e.getErrorString()).c_str(),
					ConvertedString(e.getErrorCodeAsString()).c_str() ) );
			}
			result = -urDeviceAccessError;
		}
	}
	else
	{
		result = -urDeviceAccessError;
	}
	return result;
}

//-----------------------------------------------------------------------------
void DeviceHandlerHYPERION::SelectFlashAccess( enum eFlashSelect access )
//-----------------------------------------------------------------------------
{
	std::stringstream sVal;
	ComponentLocator locator(pDev_->hDev());
	Method flashAccess(locator.findComponent("SelectFlashDevice@ii"));
	sVal.str("");
	sVal << access;
	if( flashAccess.isValid() )
		flashAccess.call(sVal.str().c_str());
}

//-----------------------------------------------------------------------------
int DeviceHandlerHYPERION::UpdateFirmware( bool boSilentMode )
//-----------------------------------------------------------------------------
{
	int result = urOperationSuccessful;
	MessageToUser( wxT("Information"), wxT("The firmware will now be updated. During this time(approx. 90 sec.) the application will not react. Please be patient."), boSilentMode, wxOK | wxICON_INFORMATION );
	wxFileDialog fileDlg( NULL, wxT("Load an existing raw programming data file"), wxT(""), wxT(""), wxT("raw-programming-data Files (*.rpd)|*.rpd"), wxOPEN | wxFILE_MUST_EXIST );
	if( fileDlg.ShowModal() == wxID_OK )
	{
		wxFFile file(fileDlg.GetPath().c_str(), wxT("rb") );
		if( !file.IsOpened() )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxString::Format( wxT("ERROR! Could not open file %s.\n"), fileDlg.GetPath().c_str() ), wxTextAttr(wxColour(255, 0, 0)) );
			}
			return urFileIOError;
		}
		unsigned char *spiData = new unsigned char [file.Length() + 0x1000];
		int bytesread = static_cast<int>(file.Read( spiData, file.Length() ));
		file.Close();
		//SelectFlashAccess( eFSSetI2CSwitchToDefaultFlash );
		//SelectFlashAccess( eFSSetI2CSwitchOffAndForceUserFlashSelect );
		CEpcs epcs( pDev_ );
		epcs.AttachParent( pParent_ );
		epcs.DeviceConfigure();
		if( epcs.IsProtected() )
		{
			if( pParent_ )
			{
				pParent_->WriteErrorMessage( wxT("ERROR! Serial flash memory is write protected, please switch S3 to position 'User' and repeat this process \n") );
			}
			return urDeviceAccessError;
		}
		epcs.BulkErase();
		int writeResult = epcs.Write( spiData, 0, bytesread, false );
		if( writeResult == 0 )
		{
			if( pParent_ )
			{
				pParent_->WriteLogMessage( wxT("Programming completed successfully.\n" ) );
				pParent_->WriteLogMessage( wxT("Please note, that the new software will only be activated after the system has been shutdown, switched off and swichted back on again. A simple reboot will NOT be sufficient.\n"), wxTextAttr(wxColour(255, 0, 0)) );
			}
			if( MessageToUser( wxT("Information"), wxT("The update task has now finished. It's recommended to verify the firmware now. Verify firmware?"), boSilentMode, wxOK | wxCANCEL | wxICON_INFORMATION ) )
			{
				int verifyResult = epcs.Verify( spiData, 0, bytesread );
				if( verifyResult != 0 )
				{
					if( pParent_ )
					{
						pParent_->WriteErrorMessage( wxString::Format( wxT("ERROR! Verify failed (result: %d).\n"), writeResult ) );
					}
					result =  urDeviceAccessError;
				}
				else if( pParent_ )
				{
					pParent_->WriteLogMessage( wxString::Format( wxT("Verify the firmware finished successfully.\n") ) );
				}
			}
		}
		else
		{
			if( pParent_ )
			{
				pParent_->WriteErrorMessage( wxString::Format( wxT("ERROR! Programming failed (result: %d).\n"), writeResult ) );
			}
			result = urDeviceAccessError;
		}
		//SelectFlashAccess( eFSSetI2CSwitchToUserFlash );
		delete [] spiData;
	}
	return result;
}
