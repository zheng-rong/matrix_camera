#include <apps/Common/Info.h>
#include <apps/Common/mvIcon.xpm>
#include <apps/Common/wxAbstraction.h>
#include "DeviceConfigureFrame.h"
#include "DeviceHandlerBlueDevice.h"
#include "DeviceHandlerBlueFOX.h"
#include "DeviceHandlerHYPERION.h"
#include "DeviceListCtrl.h"
#include <limits>
#include "LogOutputHandlerDlg.h"
#include <memory>
#include <string>
#include <wx/config.h>
#include <wx/image.h>
#include <wx/splitter.h>
#include <wx/utils.h>

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
/// \brief Allows string comparison with a defined character to ignore
///
/// This function allows a tolerant string compare. If \a candidate ends with \a wildcard
/// \a candidate can be shorter then \a searchString as the rest of the string will be
/// ignored. This is a helper function used internally by <b>DeviceManager</b> objects.
///
/// Examples:
///
///\code
/// // wildcard = '*'
/// std::string s1 = "blablabla";
/// match( s1, std::string("bl*bl*bla"), '*' ); // will return 0
/// // the next call will return -1 as the first character MUST
/// // be either a 'b' or the wildcard character.
/// match( s1, std::string("a*"), '*' );
///\endcode
/// \return
/// - 0 if successful
/// - -1 otherwise
template<class _Elem, class _Traits, class _Ax>
int match( const std::basic_string<_Elem,_Traits,_Ax>& searchString, const std::basic_string<_Elem,_Traits,_Ax>& candidate, _Elem wildcard )
//-----------------------------------------------------------------------------
{
	typename std::basic_string<_Elem,_Traits,_Ax>::size_type searchLength = searchString.length();
	// determine search length
	if( candidate.length() < searchString.length() )
	{
		if( candidate.empty() )
		{
			return -1;
		}

		if( candidate[candidate.length()-1] != wildcard )
		{
			return -1;
		}
		searchLength = candidate.length() - 1;
	}
	else if( candidate.length() > searchString.length() )
	{
		searchLength = candidate.length();
	}
	// search
	for( typename std::basic_string<_Elem,_Traits,_Ax>::size_type i = 0; i<searchLength; i++ )
	{
		if( ( candidate[i] != searchString[i] ) && ( candidate[i] != wildcard ) )
		{
			return -1;
		}
	}
	return 0;
}

//-----------------------------------------------------------------------------
class MyApp : public wxApp
//-----------------------------------------------------------------------------
{
public:
	virtual bool OnInit();
};

// ----------------------------------------------------------------------------
// event tables and other macros for wxWidgets
// ----------------------------------------------------------------------------

// the event tables connect the wxWidgets events with the functions (event
// handlers) which process them. It can be also done at run-time, but for the
// simple menu events like this the static method is much simpler.
BEGIN_EVENT_TABLE(DeviceConfigureFrame, wxFrame)
	EVT_MENU(miAbout, DeviceConfigureFrame::OnAbout)
	EVT_MENU(miQuit, DeviceConfigureFrame::OnQuit)
	EVT_MENU(miAction_SetID, DeviceConfigureFrame::OnSetID)
	EVT_MENU(miAction_UpdateFW, DeviceConfigureFrame::OnUpdateFirmware)
	EVT_MENU(miAction_UpdateKernelDriver, DeviceConfigureFrame::OnUpdateKernelDriver)
	EVT_MENU(miAction_ConfigureLogOutput, DeviceConfigureFrame::OnConfigureLogOutput)
	EVT_MENU(miAction_UpdateDeviceList, DeviceConfigureFrame::OnUpdateDeviceList)
	EVT_MENU(miAction_UpdateDMABufferSize, DeviceConfigureFrame::OnUpdateDMABufferSize)	
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
	EVT_MENU(miDirectShow_RegisterAllDevices, DeviceConfigureFrame::OnRegisterAllDevicesForDirectShow)
	EVT_MENU(miDirectShow_UnregisterAllDevices, DeviceConfigureFrame::OnUnregisterAllDevicesForDirectShow)
	EVT_MENU(miDirectShow_SetFriendlyName, DeviceConfigureFrame::OnSetFriendlyNameForDirectShow)
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
	EVT_MENU(miSettings_CPUIdleStatesEnabled, DeviceConfigureFrame::OnSettings_CPUIdleStatesEnabled)
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
	EVT_TIMER(wxID_ANY, DeviceConfigureFrame::OnTimer)
END_EVENT_TABLE()

// Create a new application object: this macro will allow wxWidgets to create
// the application object during program execution (it's better than using a
// static object for many reasons) and also declares the accessor function
// wxGetApp() which will return the reference of the right type (i.e. MyApp and
// not wxApp)
IMPLEMENT_APP(MyApp)

//-----------------------------------------------------------------------------
// `Main program' equivalent: the program execution "starts" here
bool MyApp::OnInit()
//-----------------------------------------------------------------------------
{
	// Create the main application window
	DeviceConfigureFrame *frame = new DeviceConfigureFrame(wxString::Format( wxT("Configuration tool for MATRIX VISION GmbH devices(%s)"), VERSION_STRING ), wxDefaultPosition, wxDefaultSize, argc, argv);
	frame->Show(true);
	SetTopWindow(frame);
	// success: wxApp::OnRun() will be called which will enter the main message
	// loop and the application will run. If we returned false here, the
	// application would exit immediately.
	return true;
}

//=============================================================================
//================= Implementation DeviceConfigureFrame =======================
//=============================================================================
//-----------------------------------------------------------------------------
DeviceConfigureFrame::DeviceConfigureFrame( const wxString& title, const wxPoint& pos, const wxSize& size, int argc, wxChar** argv )
	: wxFrame((wxFrame *)NULL, wxID_ANY, title, pos, size),
	m_pDevListCtrl(0), m_pLogWindow(0), m_lastDevMgrChangedCount(numeric_limits<unsigned int>::max()),
	m_customFirmwarePath(), m_IPv4Mask(), m_boPendingQuit(false)
//-----------------------------------------------------------------------------
{
	m_deviceHandlerFactory.Register( wxString(wxT("GigEVisionDevice")), DeviceHandlerBlueDevice::Create);
	m_deviceHandlerFactory.Register( wxString(wxT("mvBlueCOUGAR")), DeviceHandlerBlueDevice::Create);
	m_deviceHandlerFactory.Register( wxString(wxT("mvBlueLYNX")), DeviceHandlerBlueDevice::Create);
	m_deviceHandlerFactory.Register( wxString(wxT("mvBlueFOX")), DeviceHandlerBlueFOX::Create);
	m_deviceHandlerFactory.Register( wxString(wxT("mvBlueFOX3")), DeviceHandlerBlueDevice::Create);
	m_deviceHandlerFactory.Register( wxString(wxT("mvHYPERION")), DeviceHandlerHYPERION::Create);

	// create the menu
	/* Keyboard shortcuts:
			CTRL+C : Configure Log Output
			CTRL+F : Update Firmware
		ALT+CTRL+F : Set Friendly Name
			CTRL+I : Enable/Disable CPU idle states
			CTRL+K : Update Kernel Driver
			CTRL+P : Update Permanent DMA Buffer
			CTRL+R : Register All Devices(for DirectShow)
			CTRL+S : Set ID
			CTRL+U : Unregister All Devices(for DirectShow)
			    F5 : Update Device List
		ALT+     X : Exit
	*/

	wxMenu *menuAction = new wxMenu;
	m_pMIActionSetID = menuAction->Append( miAction_SetID, wxT("&Set ID\tCTRL+S") );
	m_pMIActionUpdateFW = menuAction->Append( miAction_UpdateFW, wxT("Update &Firmware\tCTRL+F") );
	m_pMIActionUpdateKernelDriver = menuAction->Append( miAction_UpdateKernelDriver, wxT("Update &Kernel Driver\tCTRL+K") );
	m_pMIActionUpdateDMABufferSize = menuAction->Append( miAction_UpdateDMABufferSize, wxT("Update Permanent &DMA Buffer Size\tCTRL+D") );

	menuAction->AppendSeparator();
	menuAction->Append( miAction_ConfigureLogOutput, wxT("&Configure Log Output\tCTRL+C") );
	m_pMIActionUpdateDeviceList = menuAction->Append( miAction_UpdateDeviceList, wxT("&Update Device List\tF5") );
	menuAction->AppendSeparator();

	menuAction->Append( miQuit, wxT("E&xit\tALT+X"));

	wxMenu *menuHelp = new wxMenu;
	menuHelp->Append( miAbout, wxT("About mvDeviceConfigure\tF1") );

	// create a menu bar...
	wxMenuBar *menuBar = new wxMenuBar;
	// ... add all the menu items...
	menuBar->Append( menuAction, wxT("&Action") );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
	wxMenu *menuDirectShow = new wxMenu;
	menuDirectShow->Append( miDirectShow_RegisterAllDevices, wxT("&Register All Devices\tCTRL+R") );
	menuDirectShow->Append( miDirectShow_UnregisterAllDevices, wxT("&Unregister All Devices\tCTRL+U") );
	menuDirectShow->AppendSeparator();
	m_pMIDirectShow_SetFriendlyName = menuDirectShow->Append( miDirectShow_SetFriendlyName, wxT("Set Friendly Name\tALT+CTRL+F") );
	menuBar->Append( menuDirectShow, wxT("&DirectShow") );
	m_DSDevMgr.create( this );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
	wxMenu *menuSettings = new wxMenu;
	m_pMISettings_CPUIdleStatesEnabled = menuSettings->Append( miSettings_CPUIdleStatesEnabled, wxT("CPU &Idle States Enabled\tCTRL+I"), wxT(""), wxITEM_CHECK );
	bool boValue = false;
	GetPowerState( boValue );
	m_pMISettings_CPUIdleStatesEnabled->Check( boValue );
	menuBar->Append( menuSettings, wxT("&Settings") );
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
	menuBar->Append( menuHelp, wxT("&Help") );
	// ... and attach this menu bar to the frame
	SetMenuBar(menuBar);

	// define the applications icon
	wxIcon icon(mvIcon_xpm);
	SetIcon( icon );

	wxPanel* pPanel = new wxPanel(this, wxID_ANY);
	wxSplitterWindow* pHorizontalSplitter = new wxSplitterWindow( pPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );
	wxBoxSizer* pSizer = new wxBoxSizer(wxVERTICAL);
	pSizer->Add( pHorizontalSplitter, wxSizerFlags(5).Expand() );

	m_pDevListCtrl = new DeviceListCtrl(pHorizontalSplitter, LIST_CTRL, wxDefaultPosition, wxDefaultSize, wxLC_REPORT | wxLC_SINGLE_SEL | wxSUNKEN_BORDER, this);
	m_pDevListCtrl->InsertColumn( lcFamily, wxT("Family") );
	m_pDevListCtrl->InsertColumn( lcProduct, wxT("Product") );
	m_pDevListCtrl->InsertColumn( lcSerial, wxT("Serial") );
	m_pDevListCtrl->InsertColumn( lcState, wxT("State") );
	m_pDevListCtrl->InsertColumn( lcFWVersion, wxT("Firmware Version") );
	m_pDevListCtrl->InsertColumn( lcKernelDriver, wxT("Kernel Driver") );
	m_pDevListCtrl->InsertColumn( lcDeviceID, wxT("Device ID") );
	m_pDevListCtrl->InsertColumn( lcAllocatedDMABuffer, wxT("Allocated DMA Buffer(KB)") );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
	m_pDevListCtrl->InsertColumn( lcDSRegistered, wxT("Registered For DirectShow") );
	m_pDevListCtrl->InsertColumn( lcDSFriendlyName, wxT("DirectShow Friendly Name") );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

	m_pLogWindow = new wxTextCtrl(pHorizontalSplitter, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxSUNKEN_BORDER | wxTE_RICH | wxTE_READONLY);

	// splitter...
	pHorizontalSplitter->SetSashGravity( 0.8 );
	pHorizontalSplitter->SetMinimumPaneSize( 50 );

	BuildList();

	wxString processedParameters;
	for( int i=1; i<argc; i++ )
	{
		wxString key, value;
		wxString param(argv[i]);
		key = param.BeforeFirst( wxT('=') );
		value = param.AfterFirst( wxT('=') );
		if( key.IsEmpty() )
		{
			WriteErrorMessage( wxString::Format( wxT("%s: Invalid command line parameter: '%s'.\n"), ConvertedString(__FUNCTION__).c_str(), param.c_str() ) );
		}
		else
		{
			if( ( key == wxT("update_fw") ) || ( key == wxT("ufw") ) )
			{
				GetConfigurationEntry( value )->second.boUpdateFW_ = true;
			}
			else if( key == wxT("fw_path") )
			{
				m_customFirmwarePath = value;
			}
			else if( key == wxT("ipv4_mask") )
			{
				m_IPv4Mask = value.mb_str();
			}
			else if( ( key == wxT("update_kd") ) || ( key == wxT("ukd") ) )
			{
				GetConfigurationEntry( value )->second.boUpdateKernelDriver_ = true;
			}
			else if( ( key == wxT("setid") ) || ( key == wxT("id") ) )
			{
				wxString serial(value.BeforeFirst( wxT('.') ));
				wxString idString(value.AfterFirst( wxT('.') ));
				if( serial.IsEmpty() || idString.IsEmpty() )
				{
					WriteErrorMessage( wxString::Format( wxT("%s: Invalid command line parameter: '%s'.\n"), ConvertedString(__FUNCTION__).c_str(), param.c_str() ) );
				}
				else
				{
					long id;
					if( !idString.ToLong( &id ) )
					{
						WriteErrorMessage( wxString::Format( wxT("%s: Invalid command line parameter: '%s'.\n"), ConvertedString(__FUNCTION__).c_str(), param.c_str() ) );
					}
					else
					{
						GetConfigurationEntry( serial )->second.boSetDeviceID_ = true;
						GetConfigurationEntry( serial )->second.deviceID_ = static_cast<int>(id);
					}
				}
			}
			else if( ( key == wxT("quit") ) || ( key == wxT("q") ) )
			{
				m_boPendingQuit = true;
			}
			else
			{
				WriteErrorMessage( wxString::Format( wxT("%s: Invalid command line parameter: '%s'.\n"), ConvertedString(__FUNCTION__).c_str(), param.c_str() ) );
			}
		}
		processedParameters += param;
		processedParameters.Append( wxT(' ') );
	}

	wxTextAttr boldStyle;
	m_pLogWindow->GetStyle( m_pLogWindow->GetLastPosition(), boldStyle );
	wxFont boldFont(boldStyle.GetFont());
	boldFont.SetWeight( wxFONTWEIGHT_BOLD );
	boldFont.SetPointSize( 10 );
	boldFont.SetUnderlined( true );
	boldStyle.SetFont( boldFont );
	WriteLogMessage( wxT("Available command line options:\n" ), boldStyle );
	WriteLogMessage( wxT("'setid'         or 'id'   to update the firmware of one or many devices(syntax: 'id=<serial>.id' or 'id=<product>.id')\n") );
	WriteLogMessage( wxT("'update_fw'     or 'ufw'  to update the firmware of one or many devices\n") );
	WriteLogMessage( wxT("'ipv4_mask'               to specify an IPv4 address mask to use as a filter for the selected update operations\n") );
	WriteLogMessage( wxT("'fw_path'                 to specify a custom path for the firmware files\n") );
	WriteLogMessage( wxT("'update_kd'     or 'ukd'  to update the kernel driver of one or many devices\n") );
	WriteLogMessage( wxT("'quit'          or 'q'    to end the application automatically after all updates have been applied\n") );
	WriteLogMessage( wxT("'*' can be used as a wildcard, devices will be searched by serial number AND by product. The application will first try to locate a device with a serial number matching the specified string and then (if no suitable device is found) a device with a matching product string.\n") );
	WriteLogMessage( wxT("\n") );
	WriteLogMessage( wxT("The number of commands that can be passed to the application is not limited.\n") );
	WriteLogMessage( wxT("\n") );
	WriteLogMessage( wxT("Usage examples:\n") );
	WriteLogMessage( wxT("mvDeviceConfigure ufw=BF000666 (will update the firmware of a mvBlueFOX with the serial number BF000666)\n") );
	WriteLogMessage( wxT("mvDeviceConfigure update_fw=BF* (will update the firmware of ALL mvBlueFOX devices in the current system)\n") );
	WriteLogMessage( wxT("mvDeviceConfigure update_fw=mvBlueFOX-2* (will update the firmware of ALL mvBlueFOX-2 devices in the current system.)\n") );
	WriteLogMessage( wxT("mvDeviceConfigure setid=BF000666.5 (will assign the device ID '5' to a mvBlueFOX with the serial number BF000666)\n") );
	WriteLogMessage( wxT("mvDeviceConfigure ufw=* (will update the firmware of every device in the system)\n") );
	WriteLogMessage( wxT("mvDeviceConfigure ufw=BF000666 ufw=BF000667 (will update the firmware of 2 mvBlueFOX cameras)\n") );
	WriteLogMessage( wxT("mvDeviceConfigure ipv4_mask=169.254.* update_fw=GX* (will update the firmware of all mvBlueCOUGAR-X devices with a valid IPv4 address that starts with 169.254.)\n") );
	WriteLogMessage( wxT("\n") );
	if( argc > 1 )
	{
		WriteLogMessage( wxString::Format( wxT("Processed command line: %s.\n"), processedParameters.c_str() ), boldStyle );
		WriteLogMessage( wxT("\n") );
	}
	WriteLogMessage( wxT("Usage:\n" ), boldStyle );
	wxTextAttr blueStyle(wxColour(0, 0, 255));
	WriteLogMessage( wxT("\n") );
	WriteLogMessage( wxT("Right click on a device entry to get a menu with available options for the selected device\n"), blueStyle );
	WriteLogMessage( wxT("If a menu entry is disabled the underlying feature is not available for the selected device.\n"), blueStyle );
	WriteLogMessage( wxT("\n") );
	WriteLogMessage( wxT("Double click on a list entry to start live acquisiton from this device in a new instance of wxPropView (wxPropView must be locatable via "), blueStyle );
	WriteLogMessage( wxT("the systems path or must reside in the same directory as mvDeviceConfigure!)\n"), blueStyle );
	WriteLogMessage( wxT("\n") );
	WriteLogMessage( wxT("Menu entries under 'Action' will be enabled and disabled whenever the currently selected device changes.\n"), blueStyle );
	WriteLogMessage( wxT("\n") );
	WriteLogMessage( wxT("To modify the way log outputs are created select 'Action -> Configure Log Output'.\n"), blueStyle );
	WriteLogMessage( wxT("\n") );
	if( !m_devicesToConfigure.empty() )
	{
		WriteLogMessage( wxT("Devices that await configuration:\n"), boldStyle );
		std::map<wxString, DeviceConfigurationData>::const_iterator it = m_devicesToConfigure.begin();
		const std::map<wxString, DeviceConfigurationData>::const_iterator itEND = m_devicesToConfigure.end();
		while( it != itEND )
		{
			const wxString IDString(it->second.boSetDeviceID_ ? wxString::Format( wxT("yes(%d)"), it->second.deviceID_ ) : wxT("no"));
			WriteLogMessage( wxString::Format( wxT("%s(kernel driver update: %s, firmware update: %s, assigning device ID: %s).\n"),
				it->first.c_str(),
				it->second.boUpdateKernelDriver_ ? wxT("yes") : wxT("no"),
				it->second.boUpdateFW_ ? wxT("yes") : wxT("no"),
				IDString.c_str() ) );
			++it;
		}
		WriteLogMessage( wxT("\n") );
	}

	if( !m_devicesToConfigure.empty() || m_boPendingQuit )
	{
		m_timer.SetOwner( this, teTimer );
		m_timer.Start( TIMER_PERIOD );
	}

	wxRect defaultRect(0, 0, 640, 480);
	SetSizeHints( defaultRect.GetWidth(), defaultRect.GetHeight() );
	pPanel->SetSizer( pSizer );

	// restore previous state
	wxRect rect = FramePositionStorage::Load( defaultRect );
	SetSize( rect );
	UpdateMenu( -1 );

	pHorizontalSplitter->SplitHorizontally( m_pDevListCtrl, m_pLogWindow );

	m_listUpdateTimer.SetOwner( this, teListUpdate );
	m_listUpdateTimer.Start( TIMER_PERIOD );
}

//-----------------------------------------------------------------------------
DeviceConfigureFrame::~DeviceConfigureFrame()
//-----------------------------------------------------------------------------
{
	// store the current state of the application
	FramePositionStorage::Save( this );
	if( m_listUpdateTimer.IsRunning() )
	{
		m_listUpdateTimer.Stop();
	}
	if( m_timer.IsRunning() )
	{
		m_timer.Stop();
	}
	// when we e.g. try to write config stuff on a read-only file system the result can
	// be an annoying message box. Therefore we switch off logging now, as otherwise higher level
	// clean up code might produce error messages
	wxLog::EnableLogging( false );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::ActivateDeviceInPropView( int deviceIndex )
//-----------------------------------------------------------------------------
{
	wxString commandString(wxT("wxPropView device="));
	commandString.Append(ConvertedString(m_devMgr.getDevice( deviceIndex )->serial.read()));
	commandString.Append( wxT(" live=1") );
	::wxExecute( commandString );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::BuildList( void )
//-----------------------------------------------------------------------------
{
	typedef map<int, int> IntIntMap;
	typedef pair<int, int> IntIntPair;
	typedef map<string, IntIntMap> String_IntIntMap_Map;
	typedef pair<string, IntIntMap> String_IntIntMap_Pair;
	String_IntIntMap_Map devMap;
	
	m_lastDevMgrChangedCount = m_devMgr.changedCount();
	m_pDevListCtrl->DeleteAllItems();

#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
	DirectShowDeviceManager::DSDeviceList registeredDSDevices;
	m_DSDevMgr.getDeviceList( registeredDSDevices );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

	unsigned int devCnt = m_devMgr.deviceCount();
	map<string, int> productFirmwareTable;
	for( unsigned int i=0; i<devCnt; i++ )
	{
		const string product(m_devMgr.getDevice( i )->product.read());
		const int firmwareVersion(m_devMgr.getDevice( i )->firmwareVersion.read());
		map<string, int>::iterator it = productFirmwareTable.find( product );
		if( it == productFirmwareTable.end() )
		{
			productFirmwareTable.insert( make_pair( product, firmwareVersion ) );
		}
		else if( it->second < firmwareVersion )
		{
			it->second = firmwareVersion;
		}
	}
	for( unsigned int i=0; i<devCnt; i++ )
	{
		Device* pDev = m_devMgr[i];
		long index = m_pDevListCtrl->InsertItem( i, ConvertedString(pDev->family.read()) );
		pair<String_IntIntMap_Map::iterator, bool> p = devMap.insert( String_IntIntMap_Pair(pDev->family.read(), IntIntMap()) );
		if( !p.second )
		{
			IntIntMap::iterator it = p.first->second.find( pDev->deviceID.read() );
			if( it != p.first->second.end() )
			{
				// mark items that have a conflicting device ID
				m_pDevListCtrl->SetItemTextColour( index, *wxRED );
				m_pDevListCtrl->SetItemTextColour( it->second, *wxRED );
				WriteErrorMessage( wxString::Format( wxT("WARNING: The device in row %ld(%s) has the same ID as the device in row %d(%s) belonging to the same device family. This should be resolved.\n"), index, ConvertedString(pDev->serial.read()).c_str(), it->second, ConvertedString(m_devMgr[it->second]->serial.read()).c_str() ) );
			}
			else
			{
				p.first->second.insert( IntIntPair(pDev->deviceID.read(), index) );
			}
		}
		else
		{
			p.first->second.insert( IntIntPair(pDev->deviceID.read(), index ));
		}
		const string product(pDev->product.read());
		m_pDevListCtrl->SetItem( index, lcProduct, ConvertedString(product) );
		m_pDevListCtrl->SetItem( index, lcSerial, ConvertedString(pDev->serial.read()) );
		m_pDevListCtrl->SetItem( index, lcState, ConvertedString(pDev->state.readS()) );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
		{
			DirectShowDeviceManager::DSDeviceList::const_iterator it = registeredDSDevices.find( pDev->serial.read() );
			if( it == registeredDSDevices.end() )
			{
				m_pDevListCtrl->SetItem( index, lcDSRegistered, wxString(wxT("no")) );
				m_pDevListCtrl->SetItem( index, lcDSFriendlyName, wxEmptyString );
			}
			else
			{
				m_pDevListCtrl->SetItem( index, lcDSRegistered, wxString(wxT("yes")) );
				m_pDevListCtrl->SetItem( index, lcDSFriendlyName, ConvertedString(it->second) );
			}
		}
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
		m_pDevListCtrl->SetItemData( index, i );
		if( pDev->state.read() != dsPresent )
		{
			wxColour col(wxT("LIGHT GREY"));
			m_pDevListCtrl->SetItemBackgroundColour( index, col );
		}

		{
			map<string, int>::const_iterator it = productFirmwareTable.find( product );
			bool boUpdateAvailable = false;
			if( it != productFirmwareTable.end() )
			{
				const int firmwareVersion(pDev->firmwareVersion.read());
				if( firmwareVersion < it->second )
				{
					m_pDevListCtrl->SetItemTextColour( index, *wxRED );
					WriteErrorMessage( wxString::Format( wxT("WARNING: The device in row %ld(%s) uses an outdated firmware version.\n"), index, ConvertedString(pDev->serial.read()).c_str() ) );
					boUpdateAvailable = true;
				}
			}
			m_pDevListCtrl->SetItem( index, lcFWVersion, wxString::Format( wxT("%s%s"), ConvertedString(pDev->firmwareVersion.readS()).c_str(), boUpdateAvailable ? wxT("(UPDATE AVAILABLE)") : wxT("") ) );
		}
		string kernelDriverName;
		bool boNewerDriverAvailable = false;
		bool boFeatureSupported = false;
		auto_ptr<DeviceHandler> pHandler(m_deviceHandlerFactory.CreateObject( ConvertedString(pDev->family.read()), pDev ));
		if( pHandler.get() )
		{
			boFeatureSupported = pHandler->SupportsKernelDriverUpdate( boNewerDriverAvailable, kernelDriverName );
		}
		wxString wxKernelDriverName;
		if( kernelDriverName.empty() )
		{
			wxKernelDriverName = wxString(wxT("unsupported"));
		}
		else
		{
			wxKernelDriverName = ConvertedString(kernelDriverName);
		}
		m_pDevListCtrl->SetItem( index, lcKernelDriver, wxKernelDriverName );
		int currentDMABufferSize_kB = 0;
		if( pHandler.get() && pHandler->SupportsDMABufferSizeUpdate( &currentDMABufferSize_kB ) )
		{
			m_pDevListCtrl->SetItem( index, lcAllocatedDMABuffer, wxString::Format( wxT("%d kB"), currentDMABufferSize_kB ) );
		}
		else
		{
			m_pDevListCtrl->SetItem( index, lcAllocatedDMABuffer, wxString(wxT("unsupported")) );
		}
		if( boFeatureSupported && boNewerDriverAvailable )
		{
			wxColour col(150, 75, 150);
			wxTextAttr oldKernelDriverStyle(col);
			WriteLogMessage( wxString::Format( wxT("WARNING: The device in row %ld(%s) is not running with the latest kernel driver. To update the kernel driver for each device of that family currently present right click on the device.\n"), index, ConvertedString(pDev->serial.read()).c_str() ), oldKernelDriverStyle );
			m_pDevListCtrl->SetItemBackgroundColour( index, col );
		}
		m_pDevListCtrl->SetItem( index, lcDeviceID, ConvertedString(pDev->deviceID.readS()) );
	}
	m_pDevListCtrl->SetColumnWidth( lcFamily, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
	m_pDevListCtrl->SetColumnWidth( lcProduct, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
	m_pDevListCtrl->SetColumnWidth( lcSerial, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
	m_pDevListCtrl->SetColumnWidth( lcState, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
	m_pDevListCtrl->SetColumnWidth( lcFWVersion, wxLIST_AUTOSIZE_USEHEADER );
	m_pDevListCtrl->SetColumnWidth( lcKernelDriver, wxLIST_AUTOSIZE_USEHEADER );
	m_pDevListCtrl->SetColumnWidth( lcDeviceID, wxLIST_AUTOSIZE_USEHEADER );
	m_pDevListCtrl->SetColumnWidth( lcAllocatedDMABuffer, wxLIST_AUTOSIZE_USEHEADER );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
	m_pDevListCtrl->SetColumnWidth( lcDSRegistered, wxLIST_AUTOSIZE_USEHEADER );
	m_pDevListCtrl->SetColumnWidth( lcDSFriendlyName, wxLIST_AUTOSIZE_USEHEADER );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
}

//-----------------------------------------------------------------------------
std::map<wxString, DeviceConfigureFrame::DeviceConfigurationData>::iterator DeviceConfigureFrame::GetConfigurationEntry( wxString& value )
//-----------------------------------------------------------------------------
{
	std::map<wxString, DeviceConfigurationData>::iterator it = m_devicesToConfigure.find( value );
	if( it == m_devicesToConfigure.end() )
	{
		string valueANSI(value.mb_str());
		it = m_devicesToConfigure.insert( make_pair( value, DeviceConfigurationData(valueANSI) ) ).first;
	}
	return it;
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnAbout(wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	wxBoxSizer *pTopDownSizer;
	wxDialog dlg(this, wxID_ANY, wxString(_("About mvDeviceConfigure")));
	wxIcon icon(mvIcon_xpm);
	dlg.SetIcon( icon );
	pTopDownSizer = new wxBoxSizer(wxVERTICAL);
	wxStaticText *pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("Configuration tool for %s devices"), COMPANY_NAME ) );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("(C) 2005 - %s by %s"), CURRENT_YEAR, COMPANY_NAME ) );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("Version %s"), VERSION_STRING ) );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxT("Support contact: www.matrix-vision.de") );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("This tool has been written using wxWidgets (www.wxwidgets.org) and was compiled with version %d.%d.%d of this library"), wxMAJOR_VERSION, wxMINOR_VERSION, wxRELEASE_NUMBER ) );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxT("The expat wrapper class used internally has been written by Descartes Systems Sciences, Inc.") );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("The complete source of this application can be obtained by contacting %s"), COMPANY_NAME ) );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	wxButton *pBtnOK = new wxButton(&dlg, wxID_OK, wxT("OK"));
	pBtnOK->SetDefault();
	pTopDownSizer->Add( pBtnOK, 0, wxALL | wxALIGN_RIGHT, 15 );
	dlg.SetSizer( pTopDownSizer );
	pTopDownSizer->Fit( &dlg );
	dlg.ShowModal();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnConfigureLogOutput( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	LogOutputHandlerDlg dlg(this, m_devMgr, m_debugData);
	dlg.ShowModal();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnQuit( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	// true is to force the frame to close
	Close(true);
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnSetID( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	SetID( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnTimer( wxTimerEvent& e )
//-----------------------------------------------------------------------------
{
	switch( e.GetId() )
	{
	case teListUpdate:
		if( m_lastDevMgrChangedCount == m_devMgr.changedCount() )
		{
			return;
		}
		BuildList();
		break;
	case teTimer:
		m_timer.Stop();
		if( !m_devicesToConfigure.empty() )
		{
			std::map<wxString, DeviceConfigurationData>::const_iterator it = m_devicesToConfigure.begin();
			const std::map<wxString, DeviceConfigurationData>::const_iterator itEND = m_devicesToConfigure.end();
			while( it != itEND )
			{
				int i=0;
				Device* pDev = 0;
				while( ( ( pDev = m_devMgr.getDeviceBySerial( it->second.searchToken_, i ) ) != 0 ) ||
					   ( ( pDev = m_devMgr.getDeviceByProduct( it->second.searchToken_, i ) ) != 0 ) )
				{
					if( !m_IPv4Mask.empty() )
					{
						DeviceComponentLocator locator(pDev->hDev());
						PropertyI deviceIPAddress;
						locator.bindComponent( deviceIPAddress, "DeviceIPAddress" );
						if( deviceIPAddress.isValid() )
						{
							const string IPAddress(deviceIPAddress.readS());
							if( match( IPAddress, m_IPv4Mask, '*' ) != 0 )
							{
								WriteLogMessage( wxString::Format( wxT("Device '%s' will not be configured as it IPv4 address(%s) does not match the specified mask(%s).\n"), ConvertedString(pDev->serial.read()).c_str(), ConvertedString(IPAddress).c_str(), ConvertedString(m_IPv4Mask).c_str() ) );
								++i;
								continue;
							}
						}
					}
					if( it->second.boSetDeviceID_ )
					{
						SetID( pDev, it->second.deviceID_ );
					}
					if( it->second.boUpdateKernelDriver_ )
					{
						UpdateKernelDriver( pDev, true );
					}
					if( it->second.boUpdateFW_ )
					{
						UpdateFirmware( pDev, true );
					}
					++i;
				}
				if( i == 0 )
				{
					WriteErrorMessage( wxString::Format( wxT("No device found, that matches the search criterion %s.\n"), it->first.c_str() ) );
				}
				++it;
			}
		}
		if( m_boPendingQuit )
		{
			Close( true );
		}
		break;
	default:
		break;
	}
	e.Skip();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateFirmware( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	UpdateFirmware( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateKernelDriver( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	UpdateKernelDriver( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::SetID( Device* pDev, int newID )
//-----------------------------------------------------------------------------
{
	int result = 0;
	if( newID < didlMin )
	{
		result = -4;
		WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );

	}
	else if( newID > didlMax )
	{
		result = -5;
		WriteErrorMessage( wxString::Format( wxT( "Invalid ID. Valid IDs range from %d to %d.\n" ), didlMin, didlMax ) );
	}
	else
	{
		Device* pDevByFamily = 0;
		string family = pDev->family.read();
		int devIndex = 0;
		while( ( pDevByFamily = m_devMgr.getDeviceByFamily( family, devIndex++ ) ) != 0 )
		{
			if( ( pDev != pDevByFamily ) && ( pDevByFamily->deviceID.read() == newID ) )
			{
				WriteErrorMessage( wxString::Format( wxT( "WARNING: The new ID(%d) is already assigned to at least one other device belonging to the same family(%s)(%s)????.\n" ), newID, ConvertedString(pDevByFamily->serial.read()).c_str(), ConvertedString(family).c_str() ) );
				break;
			}
		}
	}

	if( result == 0 )
	{
		WriteLogMessage( wxString::Format( wxT( "Trying to assign ID %d to %s.\n" ), newID, ConvertedString(pDev->serial.read()).c_str() ) );
		int result = pDev->setID( newID );
		if( result == DMR_FEATURE_NOT_AVAILABLE )
		{
			WriteErrorMessage( wxT( "This device doesn't support setting the device ID.\n" ) );
		}
		else if( result != DMR_NO_ERROR )
		{
			WriteErrorMessage( wxString::Format( wxT( "An error occurred: %s(please refer to the manual for this error code).\n" ), ConvertedString(ImpactAcquireException::getErrorCodeAsString( result )).c_str() ) );
		}
		else
		{
			WriteLogMessage( wxString::Format( wxT( "%s.\n" ), ConvertedString(pDev->HWUpdateResult.readS()).c_str() ) );
		}
	}
	return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::SetID( int deviceIndex )
//-----------------------------------------------------------------------------
{
	int result = 0;
	if( deviceIndex >= 0 )
	{
		Device* pDev = m_devMgr[deviceIndex];
		if( pDev )
		{
			auto_ptr<DeviceHandler> pHandler(m_deviceHandlerFactory.CreateObject( ConvertedString(pDev->family.read()), pDev ));
			if( pHandler.get() )
			{
				pHandler->AttachParent( this );
				long newID = 0;
				if( pHandler->GetIDFromUser( newID, didlMin, didlMax ) )
				{
					result = SetID( pDev, static_cast<int>(newID) );
				}
			}
		}
		else
		{
			WriteErrorMessage(wxString::Format( wxT( "Invalid item selection(index: %d).\n"), deviceIndex ) );
			result = -2;
		}
	}
	else
	{
		wxMessageDialog selectDlg(NULL, wxT("Select a device."), wxT("Error"), wxOK | wxICON_INFORMATION);
		selectDlg.ShowModal();
		result = -1;
	}
	return result;
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateDeviceList( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	UpdateDeviceList();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::UpdateDeviceList( void )
//-----------------------------------------------------------------------------
{
	WriteLogMessage( wxString(wxT("Updating device list...\n")) );
	m_devMgr.updateDeviceList();
	BuildList();
	WriteLogMessage( wxString(wxT("Done.\n")) );
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateFirmware( Device* pDev, bool boSilentMode )
//-----------------------------------------------------------------------------
{
	int result = 0;
	auto_ptr<DeviceHandler> pHandler(m_deviceHandlerFactory.CreateObject( ConvertedString(pDev->family.read()), pDev ));
	if( pHandler.get() )
	{
		wxBusyCursor busyCursorScope;
		pHandler->AttachParent( this );
		pHandler->SetCustomFirmwarePath( m_customFirmwarePath );
		result = pHandler->UpdateFirmware( boSilentMode );
		UpdateDeviceList();
	}
	else
	{
		WriteErrorMessage( wxT("This device doesn't support firmware updates.\n") );
		result = -3;
	}
	return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateFirmware( int deviceIndex )
//-----------------------------------------------------------------------------
{
	int result = 0;
	if( deviceIndex >= 0 )
	{
		Device* pDev = m_devMgr[deviceIndex];
		if( pDev )
		{
			result = UpdateFirmware( pDev, false );
		}
		else
		{
			WriteErrorMessage( wxString::Format( wxT("Invalid item selection(index: %d).\n"), deviceIndex ) );
			result = -2;
		}
	}
	else
	{
		wxMessageDialog selectDlg(NULL, wxT("Select a device."), wxT("Error"), wxOK | wxICON_INFORMATION);
		selectDlg.ShowModal();
		result = -1;
	}
	return result;
}


//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateDMABufferSize( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	UpdateDMABufferSize( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateDMABufferSize( int deviceIndex )
//-----------------------------------------------------------------------------
{
	int result = 0;
	if( deviceIndex >= 0 )
	{
		Device* pDev = m_devMgr[deviceIndex];
		if( pDev )
		{
			auto_ptr<DeviceHandler> pHandler(m_deviceHandlerFactory.CreateObject( ConvertedString(pDev->family.read()), pDev ));
			if( pHandler.get() )
			{
				wxBusyCursor busyCursorScope;
				pHandler->AttachParent( this );
				result = pHandler->UpdatePermanentDMABufferSize( false );
				//UpdateDeviceList();
			}
			else
			{
				WriteErrorMessage( wxT("This device doesn't support firmware updates.\n") );
				result = -3;
			}
		}
		else
		{
			WriteErrorMessage( wxString::Format( wxT("Invalid item selection(index: %d).\n"), deviceIndex ) );
			result = -2;
		}
	}
	else
	{
		wxMessageDialog selectDlg(NULL, wxT("Select a device."), wxT("Error"), wxOK | wxICON_INFORMATION);
		selectDlg.ShowModal();
		result = -1;
	}
	return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateKernelDriver( Device* pDev, bool boSilentMode )
//-----------------------------------------------------------------------------
{
	int result = 0;
	auto_ptr<DeviceHandler> pHandler(m_deviceHandlerFactory.CreateObject( ConvertedString(pDev->family.read()), pDev ));
	if( pHandler.get() )
	{
		wxBusyCursor busyCursorScope;
		if( m_listUpdateTimer.IsRunning() )
		{
			m_listUpdateTimer.Stop();
		}
		pHandler->AttachParent( this );
		result = pHandler->UpdateKernelDriver( boSilentMode );
		m_listUpdateTimer.Start( TIMER_PERIOD );
	}
	else
	{
		WriteErrorMessage( wxT("This device doesn't support kernel driver updates.\n") );
		result = -3;
	}
	return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateKernelDriver( int deviceIndex )
//-----------------------------------------------------------------------------
{
	int result = 0;
	if( deviceIndex >= 0 )
	{
		Device* pDev = m_devMgr[deviceIndex];
		if( pDev )
		{
			result = UpdateKernelDriver( pDev, false );
		}
		else
		{
			WriteErrorMessage( wxString::Format( wxT("Invalid item selection(index: %d).\n"), deviceIndex ) );
			result = -2;
		}
	}
	else
	{
		wxMessageDialog selectDlg(NULL, wxT("Select a device."), wxT("Error"), wxOK | wxICON_INFORMATION);
		selectDlg.ShowModal();
		result = -1;
	}
	return result;
}

#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnSetFriendlyNameForDirectShow( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	SetDSFriendlyName( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::SetDSFriendlyName( int deviceIndex )
//-----------------------------------------------------------------------------
{
	int result = 0;
	if( deviceIndex >= 0 )
	{
		Device* pDev = m_devMgr[deviceIndex];
		if( pDev )
		{
			wxListItem info;
			info.m_itemId = deviceIndex;
			info.m_col = lcDSFriendlyName;
			info.m_mask = wxLIST_MASK_TEXT;
			if( !m_pDevListCtrl->GetItem( info ) )
			{
				WriteErrorMessage( wxT( "Failed to obtain item info.\n" ) );
			}

			string devSerial = pDev->serial.read();
			string devProduct = pDev->product.read();
			WriteLogMessage( wxString::Format( wxT( "Trying to set new DirectShow friendly name for %s. Current friendly name: %s\n"), ConvertedString(devSerial).c_str(), info.m_text.c_str() ) );
			wxString newFriendlyName = wxGetTextFromUser(	wxString::Format( wxT("Enter the new DirectShow friendly name for device %s.\nMake sure that no other device is using this name already.\n"), ConvertedString(devSerial).c_str() ),
															wxT("New friendly name:"),
															wxString::Format( wxT("%s_%s"), ConvertedString(devProduct).c_str(), ConvertedString(devSerial).c_str() ),
															this );
			if( newFriendlyName == wxEmptyString )
			{
				result = -4;
				WriteErrorMessage( wxT( "Operation canceled by the user or invalid (empty) input.\n" ) );
			}
			else
			{
				// check if this name is already in use
				int itemCount = m_pDevListCtrl->GetItemCount();
				for( int i=0; i<itemCount; i++ )
				{
					info.m_itemId = i;
					info.m_col = lcDSFriendlyName;
					info.m_mask = wxLIST_MASK_TEXT;
					if( !m_pDevListCtrl->GetItem( info ) )
					{
						WriteErrorMessage( wxString::Format( wxT( "Failed to obtain item info for item %d.\n" ), i ) );
						continue;
					}
					if( info.m_text == newFriendlyName )
					{
						if( i == deviceIndex )
						{
							WriteLogMessage( wxT( "The friendly name for this device has not been changed.\n") );
							result = -5;
						}
						else
						{
							WriteErrorMessage( wxT( "WARNING: Another device is using this friendly name already. Operation skipped.\n") );
							result = -6;
						}
					}
				}

				if( result == 0 )
				{
					WriteLogMessage( wxString::Format( wxT( "Trying to assign %s as a friendly name to device %s.\n" ), newFriendlyName.c_str(), ConvertedString(devSerial).c_str() ) );
					string friendlyNameANSI(newFriendlyName.mb_str());
					result = m_DSDevMgr.registerDevice( pDev->deviceID.read(), pDev->product.read().c_str(), friendlyNameANSI.c_str(), pDev->serial.read().c_str(), pDev->family.read().c_str() );
					BuildList();
				}
			}
		}
		else
		{
			WriteErrorMessage(wxString::Format( wxT( "Invalid item selection(index: %d).\n"), deviceIndex ) );
			result = -2;
		}
	}
	else
	{
		wxMessageDialog selectDlg(NULL, wxT("Select a device."), wxT("Error"), wxOK | wxICON_INFORMATION);
		selectDlg.ShowModal();
		result = -1;
	}
	return result;
}
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::UpdateMenu( int deviceIndex )
//-----------------------------------------------------------------------------
{
	Device* pDev = 0;
	if( ( deviceIndex >= 0 ) && ( deviceIndex < static_cast<int>(m_devMgr.deviceCount()) ) )
	{
		pDev = m_devMgr.getDevice( deviceIndex );
	}
	string kernelDriverName;
	bool boNewerDriverAvailable = false;
	bool boFeatureSupported = false;
	bool boFWUpdateSupported = false;
	bool boSetIDSupported = false;
	bool boDMABufferSizeSupported = false;
	if( pDev )
	{
		auto_ptr<DeviceHandler> pHandler(m_deviceHandlerFactory.CreateObject( ConvertedString(pDev->family.read()), pDev ));
		if( pHandler.get() )
		{
			boFeatureSupported = pHandler->SupportsKernelDriverUpdate( boNewerDriverAvailable, kernelDriverName );
			boFWUpdateSupported = pHandler->SupportsFirmwareUpdate();
			boSetIDSupported = pHandler->SupportsSetID();
			boDMABufferSizeSupported = pHandler->SupportsDMABufferSizeUpdate();
		}
	}
	m_pMIActionSetID->Enable( boSetIDSupported );
	m_pMIActionUpdateFW->Enable( boFWUpdateSupported );
	m_pMIActionUpdateKernelDriver->Enable( boFeatureSupported && boNewerDriverAvailable );
	m_pMIActionUpdateDMABufferSize->Enable( boDMABufferSizeSupported );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
	m_pMIDirectShow_SetFriendlyName->Enable( pDev != 0 );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::WriteErrorMessage( const wxString& msg )
//-----------------------------------------------------------------------------
{
	wxTextAttr errorStyle( wxColour(255, 0, 0) );
	WriteLogMessage( msg, errorStyle );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::WriteLogMessage( const wxString& msg, const wxTextAttr& style /* = wxTextAttr(wxColour(0, 0, 0)) */ )
//-----------------------------------------------------------------------------
{
	if( m_pLogWindow )
	{
		long posBefore = m_pLogWindow->GetLastPosition();
		m_pLogWindow->WriteText( msg );
		long posAfter = m_pLogWindow->GetLastPosition();
		m_pLogWindow->SetStyle( posBefore, posAfter, style );
	}
}
