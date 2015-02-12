#include <algorithm>
#include <apps/Common/Info.h>
#include <apps/Common/mvIcon.xpm>
#include <apps/Common/wxAbstraction.h>
#include "AssignIPDlg.h"
#include "DeviceListCtrl.h"
#include "error_icon.xpm"
#include "IPConfigureFrame.h"
#include <limits>
#include "ok_icon.xpm"
#include <string>
#include "TLILibImports.h"
#include "wx/combobox.h"
#include "wx/config.h"
#include <wx/splitter.h>
#include <wx/spinctrl.h>
#include <wx/utils.h>

using namespace std;

#define CHECK_INTERFACE_INDEX \
	const size_t interfaceIndex = static_cast<size_t>(atoi( value.BeforeFirst( wxT(';') ).mb_str() )); \
	if( interfaceIndex >= DetecedDeviceInfo::MAX_INTERFACE_COUNT ) \
	{ \
		parserErrors.Append( wxString::Format( wxT("Invalid interface index in command line parameter: '%s'. Ignored.\n"), param.c_str() ) ); \
		continue; \
	} \

//-----------------------------------------------------------------------------
/// \brief A automatically managed buffer type to handle subscriptable data.
template<class T>
class auto_array_ptr
//-----------------------------------------------------------------------------
{
	size_t	m_cnt;
	T*		m_pBuf;
public:
	auto_array_ptr( size_t initial_size = 0 ) : m_cnt(initial_size), m_pBuf(0)
	{
		if( initial_size > 0 )
		{
			m_pBuf = new T[initial_size];
		}
	}
	/// \brief Pass ownership to the new object.
	auto_array_ptr( auto_array_ptr& rhs ) : m_cnt(rhs.m_cnt), m_pBuf(rhs.release()) {}
	/// \brief Pass ownership to the new object.
	///
	/// Do NOT delete the original T*. This is done by this class now.
	auto_array_ptr( T* pBuf, size_t cnt ) : m_cnt(cnt), m_pBuf(pBuf) {}
	~auto_array_ptr()
	{
		delete [] m_pBuf;
	}
	// functions
	operator T*() { return m_pBuf; }
	/// \brief Pass ownership. Old buffer of the left hand side object is
	/// freed, the lhs object takes ownership of the buffer of the rhs object.
	auto_array_ptr& operator=( auto_array_ptr& rhs )
	{
		if( this != &rhs )
		{
			delete [] m_pBuf;
			m_cnt = rhs.m_cnt;
			m_pBuf = rhs.release();
		}
		return *this;
	}
	/// \brief free old buffer, allocate new buffer
	void realloc( size_t newsize )
	{
		if( newsize > 0 )
		{
			delete [] m_pBuf;
			m_pBuf = new T[newsize];
			m_cnt = newsize;
		}
	}
	/// \brief increases the buffer size and keeps the old data
	void grow( size_t newsize )
	{
		if( newsize > m_cnt )
		{
			T* p = new T[newsize];
			memcpy( p, m_pBuf, m_cnt );
			delete [] m_pBuf;
			m_pBuf = p;
			m_cnt = newsize;
		}
	}
	/// \brief Release ownership, return pointer to buffer.
	T* release( void ) { T* p = m_pBuf; m_pBuf = 0; m_cnt = 0; return p; }
	/// \brief Return pointer to buffer.
	T* get( void ) { return m_pBuf; }
	/// \brief Return const pointer to buffer.
	const T* get( void ) const { return m_pBuf; }
	/// \brief Return element count.
	size_t parCnt( void ) const { return m_cnt; }
};

//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
void DeleteSecond( std::pair<_Ty1, _Ty2>& data )
//-----------------------------------------------------------------------------
{
	delete data.second;
	data.second = 0;
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
BEGIN_EVENT_TABLE(IPConfigureFrame, wxFrame)
	EVT_CLOSE(IPConfigureFrame::OnClose)
	EVT_MENU(miAbout, IPConfigureFrame::OnAbout)
	EVT_MENU(miQuit, IPConfigureFrame::OnQuit)
	EVT_MENU(miAction_AssignTemporaryIP, IPConfigureFrame::OnAssignTemporaryIP)
	EVT_MENU(miAction_UpdateDeviceList, IPConfigureFrame::OnUpdateDeviceList)
	EVT_MENU(miSettings_UseAdvancedDeviceDiscovery, IPConfigureFrame::OnUseAdvancedDeviceDiscovery)
	EVT_TEXT(widInterfaceSelector, IPConfigureFrame::OnInterfaceSelectorTextChanged)
	EVT_TEXT(widPersistentIPAddress, IPConfigureFrame::OnPersistentIPTextChanged)
	EVT_TEXT(widPersistentSubnetMask, IPConfigureFrame::OnPersistentNetmaskTextChanged)
	EVT_TEXT(widPersistentDefaultGateway, IPConfigureFrame::OnPersistentGatewayTextChanged)
	EVT_TEXT(widConnectedToIPAddress, IPConfigureFrame::OnConnectedToIPAddressTextChanged)
	EVT_BUTTON(widBtnApplyChanges, IPConfigureFrame::OnBtnApplyChanges)
	EVT_BUTTON(widBtnConfigure, IPConfigureFrame::OnBtnConfigure)
	EVT_CHECKBOX(widCBUsePersistentIP, IPConfigureFrame::OnCBUsePersistentIP)
	EVT_CHECKBOX(widCBUseDHCP, IPConfigureFrame::OnCBUseDHCP)
	EVT_TIMER(wxID_ANY, IPConfigureFrame::OnTimer)
END_EVENT_TABLE()

// Create a new application object: this macro will allow wxWidgets to create
// the application object during program execution (it's better than using a
// static object for many reasons) and also declares the accessor function
// wxGetApp() which will return the reference of the right type (i.e. MyApp and
// not wxApp)
IMPLEMENT_APP(MyApp)

IPConfigureFrame* g_pFrame = 0;

//-----------------------------------------------------------------------------
// `Main program' equivalent: the program execution "starts" here
bool MyApp::OnInit()
//-----------------------------------------------------------------------------
{
	// Create the main application window
	g_pFrame = new IPConfigureFrame(wxString::Format( wxT("Configuration tool for GigE Vision(tm) devices(%s)"), VERSION_STRING ), wxDefaultPosition, wxDefaultSize, argc, argv);
	g_pFrame->Show( true );
	SetTopWindow( g_pFrame );
	// success: wxApp::OnRun() will be called which will enter the main message
	// loop and the application will run. If we returned false here, the
	// application would exit immediately.
	return true;
}

//=============================================================================
//================= Implementation IPConfigureFrame ===========================
//=============================================================================
const wxTextAttr IPConfigureFrame::m_ERROR_STYLE(wxColour(255, 0, 0));

//-----------------------------------------------------------------------------
IPConfigureFrame::IPConfigureFrame( const wxString& title, const wxPoint& pos, const wxSize& size, int argc, wxChar** argv )
	: wxFrame((wxFrame *)NULL, wxID_ANY, title, pos, size), m_pLogWindow(0), m_pTCPersistentIPAddress(0),
	m_pTCPersistentSubnetMask(0), m_pTCPersistentDefaultGateway(0), m_hTLI(0), m_TLILib()
//-----------------------------------------------------------------------------
{
	wxMenu *menuAction = new wxMenu;
	menuAction->Append( miAction_AssignTemporaryIP, wxT("&Assign Temporary IPv4 Address\tCTRL+A") );

	menuAction->AppendSeparator();
	menuAction->Append( miAction_UpdateDeviceList, wxT("Update Device List\tF5") );

	menuAction->AppendSeparator();
	menuAction->Append( miQuit, wxT("E&xit\tALT+X"));

	wxMenu *menuSettings = new wxMenu;
	m_pMISettings_UseAdvancedDeviceDiscovery = menuSettings->Append( miSettings_UseAdvancedDeviceDiscovery, wxT("Use Advanced Device Discovery"), wxT(""), wxITEM_CHECK );

	wxMenu *menuHelp = new wxMenu;
	menuHelp->Append( miAbout, wxT("About mvIPConfigure\tF1"));

	wxMenuBar *menuBar = new wxMenuBar;
	menuBar->Append( menuAction, wxT("&Action") );
	menuBar->Append( menuSettings, wxT("&Settings") );
	menuBar->Append( menuHelp, wxT("&Help") );
	// ... and attach this menu bar to the frame
	SetMenuBar(menuBar);

	// define the applications icon
	wxIcon icon(mvIcon_xpm);
	SetIcon( icon );

	m_iconList.Create( 16, 16, true, 0 );
	m_iconList.Add( wxIcon(error_icon_xpm) );
	m_iconList.Add( wxIcon(ok_icon_xpm) );

	wxPanel* pPanel = new wxPanel(this);

	// splitter for log window and the set of controls in the upper part of the dialog
	m_pHorizontalSplitter = new wxSplitterWindow( pPanel, widHorSplitter, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );
	m_pHorizontalSplitter->SetMinimumPaneSize( 25 );

	// splitter for device list and the device info window
	m_pVerticalSplitter = new wxSplitterWindow( m_pHorizontalSplitter, widVerSplitter, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );
	m_pVerticalSplitter->SetMinimumPaneSize( 270 );

	// device list on the left side of the splitter
	m_pDevListCtrl = new DeviceListCtrl( m_pVerticalSplitter, LIST_CTRL, wxDefaultPosition, wxDefaultSize, wxLC_REPORT | wxLC_SINGLE_SEL | wxSUNKEN_BORDER, this );
	m_pDevListCtrl->InsertColumn( lcProduct, wxT("Product") );
	m_pDevListCtrl->InsertColumn( lcSerial, wxT("Serial") );
	m_pDevListCtrl->InsertColumn( lcPrimaryInterfaceIPAddress, wxT("IP Address(Primary Interface)") );
	m_pDevListCtrl->SetImageList( &m_iconList, wxIMAGE_LIST_SMALL );

	// and a new panel for the device info and controls on the right
	wxScrolledWindow* pControlsPanel = new wxScrolledWindow(m_pVerticalSplitter);
	pControlsPanel->SetScrollRate( 10, 10 );

	const int GROUPBOX_BORDER_WIDTH_PIXEL = 5;
	const int BTN_BORDER_WIDTH_PIXEL = 4;
	const int CHECKBOX_BORDER_PIXEL_WIDTH = 3;
	
	// device information controls
	wxBoxSizer* pDeviceInfoSizer = new wxStaticBoxSizer(wxVERTICAL, pControlsPanel, wxT("Device Information: "));
	wxFlexGridSizer* pDeviceInfoElementsGridSizer = new wxFlexGridSizer(2, 0);
	pDeviceInfoElementsGridSizer->AddGrowableCol( 1, 3 );

	// row 1
	pDeviceInfoElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Manufacturer: ")), wxSizerFlags().Left() );
	m_pSTManufacturer = new wxStaticText(pControlsPanel, widDeviceManufacturer, wxT("-"), wxDefaultPosition, wxDefaultSize, wxST_NO_AUTORESIZE);
	m_pSTManufacturer->Wrap(-1);
	pDeviceInfoElementsGridSizer->Add( m_pSTManufacturer, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 2
	pDeviceInfoElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Serial Number: ")), wxSizerFlags().Left() );
	m_pSTSerialNumber = new wxStaticText(pControlsPanel, widDeviceSerial, wxT("-"));
	pDeviceInfoElementsGridSizer->Add( m_pSTSerialNumber, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 3
	pDeviceInfoElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("User Defined Name: ")), wxSizerFlags().Left() );
	m_pTCUserDefinedName = new wxTextCtrl(pControlsPanel, wxID_ANY);
	pDeviceInfoElementsGridSizer->Add( m_pTCUserDefinedName, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 4
	pDeviceInfoElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Interface Count: ")), wxSizerFlags().Left() );
	m_pSTInterfaceCount = new wxStaticText(pControlsPanel, widDeviceInterfaceCount, wxT("-"));
	pDeviceInfoElementsGridSizer->Add( m_pSTInterfaceCount, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );

	pDeviceInfoSizer->Add( pDeviceInfoElementsGridSizer, wxSizerFlags().Align( wxGROW ) );

	// interface selector
	wxBoxSizer* pInterfaceSelector = new wxBoxSizer(wxHORIZONTAL);
	pInterfaceSelector->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Selected Interface:") ));
	m_pSCInterfaceSelector = new wxSpinCtrl(pControlsPanel, widInterfaceSelector, wxT("0"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 3, 0);
	pInterfaceSelector->Add( m_pSCInterfaceSelector, wxSizerFlags().Left() );

	// current IP address controls
	wxBoxSizer* pCurrentIPSizer = new wxStaticBoxSizer(wxVERTICAL, pControlsPanel, wxT("Current Interface Parameter: "));
	// device information controls
	wxFlexGridSizer* pCurrentIPElementsGridSizer = new wxFlexGridSizer(2, 0);
	pCurrentIPElementsGridSizer->AddGrowableCol( 1, 3 );

	// row 1
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("IPv4 Address: ")), wxSizerFlags().Left() );
	m_pSTCurrentIPAddress = new wxStaticText(pControlsPanel, wxID_ANY, wxT("-"));
	pCurrentIPElementsGridSizer->Add( m_pSTCurrentIPAddress, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 2
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Subnet Mask: ")), wxSizerFlags().Left() );
	m_pSTCurrentSubnetMask = new wxStaticText(pControlsPanel, wxID_ANY, wxT("-"));
	pCurrentIPElementsGridSizer->Add( m_pSTCurrentSubnetMask, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 3
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Default Gateway: ")), wxSizerFlags().Left() );
	m_pSTCurrentDefaultGateway = new wxStaticText(pControlsPanel, wxID_ANY, wxT("-"));
	pCurrentIPElementsGridSizer->Add( m_pSTCurrentDefaultGateway, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 4
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("MAC Address: ")), wxSizerFlags().Left() );
	m_pSTMACAddress = new wxStaticText(pControlsPanel, wxID_ANY, wxT("-"));
	pCurrentIPElementsGridSizer->Add( m_pSTMACAddress, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 5
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Connected To IPv4 Address: ")), wxSizerFlags().Left() );
	m_pCBConnectedToIPAddress = new wxComboBox(pControlsPanel, widConnectedToIPAddress, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, 0, wxCB_DROPDOWN);
	m_pCBConnectedToIPAddress->Append( wxT("-") );
	m_pCBConnectedToIPAddress->Select( 0 );
	pCurrentIPElementsGridSizer->Add( m_pCBConnectedToIPAddress, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 6
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Connected Adapter Netmask: ")), wxSizerFlags().Left() );
	m_pSTConnectedToNetmask = new wxStaticText(pControlsPanel, wxID_ANY, wxT("-"));
	pCurrentIPElementsGridSizer->Add( m_pSTConnectedToNetmask, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 7
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Connected Adapter MTU(Bytes): ")), wxSizerFlags().Left() );
	m_pSTConnectedToMTU = new wxStaticText(pControlsPanel, wxID_ANY, wxT("-"));
	pCurrentIPElementsGridSizer->Add( m_pSTConnectedToMTU, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 8
	pCurrentIPElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Connected Adapter Link Speed(MBps): ")), wxSizerFlags().Left() );
	m_pSTConnectedToLinkSpeed = new wxStaticText(pControlsPanel, wxID_ANY, wxT("-"));
	pCurrentIPElementsGridSizer->Add( m_pSTConnectedToLinkSpeed, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );

	pCurrentIPSizer->Add( pCurrentIPElementsGridSizer, wxSizerFlags().Align( wxGROW ) );

	// persistent IP address related controls
	wxBoxSizer* pPersistentIPSizer = new wxStaticBoxSizer(wxVERTICAL, pControlsPanel, wxT("Persistent IPv4 Address: "));
	wxFlexGridSizer* pPersistentIPEditElementsGridSizer = new wxFlexGridSizer(2, 0);
	pPersistentIPEditElementsGridSizer->AddGrowableCol( 1, 3 );

	// row 1
	pPersistentIPEditElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("IPv4 Address: ")), wxSizerFlags().Left() );
	m_pTCPersistentIPAddress = new wxTextCtrl(pControlsPanel, widPersistentIPAddress, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, m_IPv4StringValidator);
	pPersistentIPEditElementsGridSizer->Add( m_pTCPersistentIPAddress, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 2
	pPersistentIPEditElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Subnet Mask: ")), wxSizerFlags().Left() );
	m_pTCPersistentSubnetMask = new wxTextCtrl(pControlsPanel, widPersistentSubnetMask, wxT("255.255.255.0"), wxDefaultPosition, wxDefaultSize, 0, m_IPv4StringValidator);
	pPersistentIPEditElementsGridSizer->Add( m_pTCPersistentSubnetMask, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 3
	pPersistentIPEditElementsGridSizer->Add( new wxStaticText(pControlsPanel, wxID_ANY, wxT("Default Gateway: ")), wxSizerFlags().Left() );
	m_pTCPersistentDefaultGateway = new wxTextCtrl(pControlsPanel, widPersistentDefaultGateway, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, m_IPv4StringValidator);
	pPersistentIPEditElementsGridSizer->Add( m_pTCPersistentDefaultGateway, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );

	pPersistentIPSizer->Add( pPersistentIPEditElementsGridSizer, wxSizerFlags().Align( wxGROW ) );

	// IP configuration related controls
	wxBoxSizer* pIPConfigurationSizer = new wxStaticBoxSizer(wxVERTICAL, pControlsPanel, wxT("IP Configuration: "));
	m_pCBUsePersistentIP = new wxCheckBox(pControlsPanel, widCBUsePersistentIP, wxT("Use Persistent IP"));
	pIPConfigurationSizer->Add( m_pCBUsePersistentIP, wxSizerFlags().Expand().Border( wxALL, CHECKBOX_BORDER_PIXEL_WIDTH ) );
	m_pCBUseDHCP = new wxCheckBox(pControlsPanel, widCBUseDHCP, wxT("Use DHCP"));
	pIPConfigurationSizer->Add( m_pCBUseDHCP, wxSizerFlags().Expand().Border( wxALL, CHECKBOX_BORDER_PIXEL_WIDTH ) );
	m_pCBUseLLA = new wxCheckBox(pControlsPanel, wxID_ANY, wxT("Use LLA"));
	pIPConfigurationSizer->Add( m_pCBUseLLA, wxSizerFlags().Expand().Border( wxALL, CHECKBOX_BORDER_PIXEL_WIDTH ) );
	m_pCBUseLLA->Enable( false );
	m_pCBUseLLA->SetValue( true );

	// interface configuration controls
	wxBoxSizer* pInterfaceConfigurationSizer = new wxStaticBoxSizer(wxVERTICAL, pControlsPanel, wxT("Interface Configuration: "));
	pInterfaceConfigurationSizer->Add( pInterfaceSelector, wxSizerFlags().Expand().Border( wxALL, GROUPBOX_BORDER_WIDTH_PIXEL ) );
	pInterfaceConfigurationSizer->Add( pCurrentIPSizer, wxSizerFlags().Expand().Border( wxALL, GROUPBOX_BORDER_WIDTH_PIXEL ) );
	pInterfaceConfigurationSizer->Add( pPersistentIPSizer, wxSizerFlags().Expand().Border( wxALL, GROUPBOX_BORDER_WIDTH_PIXEL ) );
	pInterfaceConfigurationSizer->Add( pIPConfigurationSizer, wxSizerFlags().Expand().Border( wxALL, GROUPBOX_BORDER_WIDTH_PIXEL ) );

	wxBoxSizer* pButtonSizer = new wxBoxSizer(wxHORIZONTAL);
	m_pBtnConfigure = new wxButton(pControlsPanel, widBtnConfigure, wxT("&Configure"));
	pButtonSizer->Add( m_pBtnConfigure, wxSizerFlags().Right().Border( wxALL, BTN_BORDER_WIDTH_PIXEL ) );
	m_pBtnApplyChanges = new wxButton(pControlsPanel, widBtnApplyChanges, wxT("&Apply Changes"));
	pButtonSizer->Add( m_pBtnApplyChanges, wxSizerFlags().Right().Border( wxALL, BTN_BORDER_WIDTH_PIXEL ) );

	wxBoxSizer* pControlsSizer = new wxBoxSizer(wxVERTICAL);
	pControlsSizer->Add( pDeviceInfoSizer, wxSizerFlags().Expand().Border( wxALL, GROUPBOX_BORDER_WIDTH_PIXEL ) );
	pControlsSizer->Add( pInterfaceConfigurationSizer, wxSizerFlags().Expand().Border( wxALL, GROUPBOX_BORDER_WIDTH_PIXEL ) );
	pControlsSizer->Add( pButtonSizer, wxSizerFlags().Right() );
	pControlsSizer->AddSpacer( 25 );

	m_pLogWindow = new wxTextCtrl(m_pHorizontalSplitter, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxSUNKEN_BORDER | wxTE_RICH | wxTE_READONLY);
	
	wxBoxSizer* pSizer = new wxBoxSizer(wxVERTICAL);
	pSizer->Add( m_pHorizontalSplitter, wxSizerFlags(1).Expand() );

	wxRect defaultRect(0, 0, 1024, 768);
	pPanel->SetSizer( pSizer );
	// restore previous state
	wxConfigBase* pConfig = wxConfigBase::Get();
	wxRect rect = FramePositionStorage::Load( defaultRect );
	m_pMISettings_UseAdvancedDeviceDiscovery->Check( pConfig->Read( wxT("/MainFrame/useAdvancedDeviceDiscovery"), 1l ) != 0 );
	int verticalSplitterPos = pConfig->Read( wxT("/MainFrame/verticalSplitter"), -1l );
	int horizontalSplitterPos = pConfig->Read( wxT("/MainFrame/horizontalSplitter"), -1l );
	
	m_pVerticalSplitter->SplitVertically( m_pDevListCtrl, pControlsPanel, 0 );
	m_pHorizontalSplitter->SplitHorizontally( m_pVerticalSplitter, m_pLogWindow, 0 );

	pControlsPanel->SetSizer( pControlsSizer );
	SetClientSize( pSizer->GetMinSize() );
	pSizer->SetSizeHints( this );
	SetSize( rect );

	m_pVerticalSplitter->SetSashPosition( ( verticalSplitterPos != -1 ) ? verticalSplitterPos : 2*rect.width/5, true );
	m_pHorizontalSplitter->SetSashPosition( ( horizontalSplitterPos != -1 ) ? horizontalSplitterPos : 4*rect.height/7, true );

	const wxString GenTLLibLoadMessage(LoadGenTLProducer( m_TLILib ));
	if( !GenTLLibLoadMessage.IsEmpty() )
	{
		WriteLogMessage( GenTLLibLoadMessage );
	}

	if( m_TLILib.IsLoaded() )
	{
		EXTRACT_SYMBOL( TLOpen, WriteLogMessage )
		EXTRACT_SYMBOL( TLClose, WriteLogMessage )
		EXTRACT_SYMBOL( TLUpdateInterfaceList, WriteLogMessage )
		EXTRACT_SYMBOL( TLGetNumInterfaces, WriteLogMessage )
		EXTRACT_SYMBOL( TLGetInterfaceID, WriteLogMessage )
		EXTRACT_SYMBOL( TLOpenInterface, WriteLogMessage )
		EXTRACT_SYMBOL( IFClose, WriteLogMessage )
		EXTRACT_SYMBOL( IFGetNumDevices, WriteLogMessage )
		EXTRACT_SYMBOL( IFGetDeviceID, WriteLogMessage )
		EXTRACT_SYMBOL( IFGetInfo, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_IFSetInterfaceParam, WriteLogMessage )
		EXTRACT_SYMBOL( IFUpdateDeviceList, WriteLogMessage )
		EXTRACT_SYMBOL( IFGetDeviceInfo, WriteLogMessage )
		EXTRACT_SYMBOL( IFOpenDevice, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_IFGetDeviceInterfaceInfo, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_DevSetInterfaceParam, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_DevSetParam, WriteLogMessage )
		EXTRACT_SYMBOL( DevClose, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_MACFromSerial, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_IsValidIPv4Address, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_DoAdressesMatch, WriteLogMessage )
		EXTRACT_SYMBOL( TLIMV_ForceIP, WriteLogMessage )
	}

	wxTextAttr boldStyle;
	m_pLogWindow->GetStyle( m_pLogWindow->GetLastPosition(), boldStyle );
	wxFont boldFont(boldStyle.GetFont());
	boldFont.SetWeight( wxFONTWEIGHT_BOLD );
	boldFont.SetPointSize( 10 );
	boldFont.SetUnderlined( true );
	boldStyle.SetFont( boldFont );
	WriteLogMessage( wxT("Available command line options:\n" ), boldStyle );
	WriteLogMessage( wxT("'device' or 'd' to select a device for configuration\n") );
	WriteLogMessage( wxT("'userDefinedName' or 'udn' to set a user defined name for the device currently selected\n") );
	WriteLogMessage( wxT("'useDHCP' to enable/disable the usage of DHCP for the device currently selected(value syntax: <interface index>;<value>)\n") );
	WriteLogMessage( wxT("'usePersistentIP' to enable/disable the usage of a persistent IP address for the device currently selected(value syntax: <interface index>;<value>)\n") );
	WriteLogMessage( wxT("'persistentIPAddress' to define a persistent IP address for the device currently selected(value syntax: <interface index>;<value>)\n") );
	WriteLogMessage( wxT("'persistentSubnetMask' to define a persistent subnet mask for the device currently selected(value syntax: <interface index>;<value>)\n") );
	WriteLogMessage( wxT("'persistentDefaultGateway' to define a persistent default gateway for the device currently selected(value syntax: <interface index>;<value>)\n") );
	WriteLogMessage( wxT("'quit' or 'q' to automatically terminate the application after all the configuration has been applied\n") );
	WriteLogMessage( wxT("\n") );
	WriteLogMessage( wxT("Usage examples:\n") );
	WriteLogMessage( wxT("mvIPConfigure device=GX000066 usePersistentIP=0;1 persistentIPAddress=0;172.111.2.1 persistentSubnetMask=0;255.255.255.0 persistentDefaultGateway=0;172.111.2.2 quit\n") );
	WriteLogMessage( wxT("\n") );

	int status = 0;
	LOGGED_TLI_CALL( TLOpen, ( &m_hTLI ), WriteLogMessage )
	UpdateDeviceList();

	wxString parserErrors;
	wxString processedParameters;
	wxString deviceToConfigure;
	wxString userDefinedName;
	bool boMustQuit = false;
	InterfaceInfo interfaceInfo[DetecedDeviceInfo::MAX_INTERFACE_COUNT];
	for( int i=1; i<argc; i++ )
	{
		wxString key, value;
		wxString param(argv[i]);
		key = param.BeforeFirst( wxT('=') );
		value = param.AfterFirst( wxT('=') );
		if( key.IsEmpty() )
		{
			parserErrors.Append( wxString::Format( wxT("Invalid command line parameter: '%s'. Ignored.\n"), param.c_str() ) );
		}
		else
		{
			if( ( key == wxT("device") ) || ( key == wxT("d") ) )
			{
				if( !deviceToConfigure.IsEmpty() )
				{
					for( size_t j=0; j<DetecedDeviceInfo::MAX_INTERFACE_COUNT; j++ )
					{
						m_interfaceInfo[j] = interfaceInfo[j];
					}
					ApplyChanges( deviceToConfigure, m_pDevListCtrl->GetItemText( m_pDevListCtrl->GetCurrentItemIndex() ), m_pCBConnectedToIPAddress->GetValue(), userDefinedName );
				}
				deviceToConfigure = wxEmptyString;
				if( SelectDevice( value ) )
				{
					deviceToConfigure = value;
					for( size_t j=0; j<DetecedDeviceInfo::MAX_INTERFACE_COUNT; j++ )
					{
						interfaceInfo[j] = m_interfaceInfo[j];
					}
				}
			}
			else if( ( key == wxT("userDefinedName") ) || ( key == wxT("udn") ) )
			{
				userDefinedName = value;
			}
			else if( key == wxT("useDHCP") )
			{
				CHECK_INTERFACE_INDEX;
				interfaceInfo[interfaceIndex].DHCPEnabled_ = atoi( value.AfterLast( wxT(';') ).mb_str() ) != 0;
			}
			else if( key == wxT("usePersistentIP") )
			{
				CHECK_INTERFACE_INDEX;
				interfaceInfo[interfaceIndex].persistentIPEnabled_ = atoi( value.AfterLast( wxT(';') ).mb_str() ) != 0;
			}
			else if( key == wxT("persistentIPAddress") )
			{
				CHECK_INTERFACE_INDEX;
				interfaceInfo[interfaceIndex].persistentIPAddress_ = value.AfterLast( wxT(';') ).mb_str();
			}
			else if( key == wxT("persistentSubnetMask") )
			{
				CHECK_INTERFACE_INDEX;
				interfaceInfo[interfaceIndex].persistentSubnetMask_ = value.AfterLast( wxT(';') ).mb_str();
			}
			else if( key == wxT("persistentDefaultGateway") )
			{
				CHECK_INTERFACE_INDEX;
				interfaceInfo[interfaceIndex].persistentDefaultGateway_ = value.AfterLast( wxT(';') ).mb_str();
			}
			else if( ( key == wxT("quit") ) || ( key == wxT("q") ) )
			{
				boMustQuit = true;
			}
			else
			{
				parserErrors.Append( wxString::Format( wxT("Invalid command line parameter: '%s'. Ignored.\n"), param.c_str() ) );
			}
			processedParameters += param;
			processedParameters.Append( wxT(' ') );
		}
	}

	if( !deviceToConfigure.IsEmpty() )
	{
		for( size_t j=0; j<DetecedDeviceInfo::MAX_INTERFACE_COUNT; j++ )
		{
			m_interfaceInfo[j] = interfaceInfo[j];
		}
		ApplyChanges( deviceToConfigure, m_pDevListCtrl->GetItemText( m_pDevListCtrl->GetCurrentItemIndex() ), m_pCBConnectedToIPAddress->GetValue(), userDefinedName );
	}

	WriteLogMessage( wxT("\n") );
	const wxString none(wxT("none"));
	WriteLogMessage( wxString::Format( wxT("Processed command line parameters: %s\n"), ( processedParameters.length() > 0 ) ? processedParameters.c_str() : none.c_str() ), boldStyle );
	//WriteLogMessage( wxString::Format( wxT("Processed command line parameters: %s\n"), ( processedParameters.length() > 0 ) ? processedParameters.c_str() : wxT("none") ), boldStyle ); // will cause a 'deprecated conversion from string constant to 'char*' on some platform/wxWidgets combinations
	WriteLogMessage( wxT("\n") );
	if( !parserErrors.IsEmpty() )
	{
		WriteLogMessage( parserErrors, m_ERROR_STYLE );
		WriteLogMessage( wxT("\n") );
	}

	if( boMustQuit )
	{
		m_quitTimer.SetOwner( this, teQuit );
		m_quitTimer.Start( 1000 );
	}
}

//-----------------------------------------------------------------------------
bool IPConfigureFrame::SelectDevice( const wxString& deviceToConfigure )
//-----------------------------------------------------------------------------
{
	if( !deviceToConfigure.IsEmpty() )
	{
		const int cnt = m_pDevListCtrl->GetItemCount();
		for( int i=0; i<cnt; i++ )
		{
			wxListItem info;
			info.m_itemId = i;
			info.m_col = lcSerial;
			info.m_mask = wxLIST_MASK_TEXT;
			if( m_pDevListCtrl->GetItem( info ) )
			{
				if( info.m_text == deviceToConfigure )
				{
					m_pDevListCtrl->SetCurrentItemIndex( i );
					UpdateDlgControls( true );
					return true;
				}
			}
		}
	}
	return false;
}

//-----------------------------------------------------------------------------
IPConfigureFrame::~IPConfigureFrame()
//-----------------------------------------------------------------------------
{
	Deinit();
	{
		// store the current state of the application
		FramePositionStorage::Save( this );
		// when we e.g. try to write config stuff on a read-only file system the result can
		// be an annoying message box. Therefore we switch off logging during the storage operation.
		wxLogNull logSuspendScope;
		wxConfigBase* pConfig = wxConfigBase::Get();
		pConfig->Write( wxT("/MainFrame/useAdvancedDeviceDiscovery"), m_pMISettings_UseAdvancedDeviceDiscovery->IsChecked() );
		pConfig->Write( wxT("/MainFrame/verticalSplitter"), m_pVerticalSplitter->GetSashPosition() );
		pConfig->Write( wxT("/MainFrame/horizontalSplitter"), m_pHorizontalSplitter->GetSashPosition() );
		pConfig->Flush();
	}
	InterfaceContainer::iterator it = m_TLIInterfaces.begin();
	InterfaceContainer::iterator itEnd = m_TLIInterfaces.end();
	int status = 0;
	while( it != itEnd )
	{
		LOGGED_TLI_CALL( IFClose, ( it->second ), WriteLogMessage )
		++it;
	}
	LOGGED_TLI_CALL( TLClose, ( m_hTLI ), WriteLogMessage )
	for_each( m_devices.begin(), m_devices.end(), ptr_fun(DeleteSecond<const string, DetecedDeviceInfo*>) );
	m_devices.clear();
	// when we e.g. try to write config stuff on a read-only file system the result can
	// be an annoying message box. Therefore we switch off logging now, as otherwise higher level
	// clean up code might produce error messages
	wxLog::EnableLogging( false );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::Deinit( void )
//-----------------------------------------------------------------------------
{
	if( m_quitTimer.IsRunning() )
	{
		m_quitTimer.Stop();
	}
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnBtnApplyChanges( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	int currentItem = m_pDevListCtrl->GetCurrentItemIndex();
	if( currentItem < 0 )
	{
		WriteLogMessage( wxT("ERROR: No device selected.\n"), m_ERROR_STYLE );
		return;
	}

	wxString itemText(m_pDevListCtrl->GetItemText( currentItem ));
	wxListItem info;
	info.m_itemId = currentItem;
	info.m_col = lcSerial;
	info.m_mask = wxLIST_MASK_TEXT;
	if( !m_pDevListCtrl->GetItem( info ) )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR: Could not obtain serial number for device %s.\n"), itemText.c_str() ) );
		return;
	}

	ApplyChanges( info.m_text, itemText, m_pCBConnectedToIPAddress->GetValue(), m_pTCUserDefinedName->GetValue() );
}


//-----------------------------------------------------------------------------
void IPConfigureFrame::ApplyChanges( const wxString& serial, const wxString& product, const wxString& connectedToIPAddress, const wxString& userDefinedName )
//-----------------------------------------------------------------------------
{
	InterfaceContainer::const_iterator itInterface = m_TLIInterfaces.begin();
	InterfaceContainer::const_iterator itInterfaceEND = m_TLIInterfaces.end();
	while( itInterface != itInterfaceEND )
	{
		wxString adapterIPAddress(ConvertedString(GetInterfaceStringInfo( itInterface->second, INTERFACE_INFO_IP_STRING )));
		if( adapterIPAddress == connectedToIPAddress )
		{
			break;
		}
		++itInterface;
	}

	if( itInterface == m_TLIInterfaces.end() )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR: Could not obtain interface handle to adapter %s.\n"), connectedToIPAddress.c_str() ) );
		return;
	}

	DeviceMap::const_iterator itDev = m_devices.find( string(m_pSTSerialNumber->GetLabel().mb_str()) );
	if( itDev == m_devices.end() )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR: Could not obtain device name for device %s on adapter %s.\n"), serial.c_str(), connectedToIPAddress.c_str() ) );
		return;
	}

	int status = 0;
	for( unsigned int i=0; i<itDev->second->interfaceCount_; i++ )
	{
		if( itDev->second->interfaceInfo_[i].supportsDHCP_ )
		{
			if( m_interfaceInfo[i].persistentIPEnabled_ )
			{
				wxString IPAddress(ConvertedString(m_interfaceInfo[i].persistentIPAddress_));
				wxString SubnetMask(ConvertedString(m_interfaceInfo[i].persistentSubnetMask_));
				wxString Gateway(ConvertedString(m_interfaceInfo[i].persistentDefaultGateway_));
				if( !ValidateIPDataSet( IPAddress, SubnetMask, Gateway, this, this ) )
				{
					return;
				}
			}
		}
	}

	WriteLogMessage( wxString::Format( wxT("Trying to establish write access to device %s(%s).\n"), serial.c_str(), product.c_str() ) );
	MVTLI_DEVICE_HANDLE hDev = 0;
	LOGGED_TLI_CALL( IFOpenDevice, ( itInterface->second, itDev->second->deviceName_.c_str(), DEVICE_ACCESS_EXCLUSIVE, &hDev ), WriteLogMessage )
	if( status != 0 )
	{
		return;
	}

	unsigned int timeout_ms = 2000;
	LOGGED_TLI_CALL( TLIMV_DevSetParam, ( hDev, DEVICE_INFO_GVCP_MESSAGE_TIMEOUT, &timeout_ms, sizeof(timeout_ms) ), WriteLogMessage )

	WriteLogMessage( wxString::Format( wxT("Trying to apply changes to device %s(%s).\n"), serial.c_str(), product.c_str() ) );
	for( unsigned int i=0; i<itDev->second->interfaceCount_; i++ )
	{
		if( itDev->second->interfaceInfo_[i].supportsDHCP_ )
		{
			LOGGED_TLI_CALL( TLIMV_DevSetInterfaceParam, ( hDev, i, DEVICE_INFO_CURRENT_IP_DHCP, &m_interfaceInfo[i].DHCPEnabled_, sizeof(m_interfaceInfo[i].DHCPEnabled_) ), WriteLogMessage )
		}

		if( itDev->second->interfaceInfo_[i].supportsPersistentIP_ )
		{
			LOGGED_TLI_CALL( TLIMV_DevSetInterfaceParam, ( hDev, i, DEVICE_INFO_CURRENT_IP_PERSISTENT, &m_interfaceInfo[i].persistentIPEnabled_, sizeof(m_interfaceInfo[i].persistentIPEnabled_) ), WriteLogMessage )
			LOGGED_TLI_CALL( TLIMV_DevSetInterfaceParam, ( hDev, i, DEVICE_INFO_PERSISTENT_IP_STRING, m_interfaceInfo[i].persistentIPAddress_.c_str(), m_interfaceInfo[i].persistentIPAddress_.length() ), WriteLogMessage )
			LOGGED_TLI_CALL( TLIMV_DevSetInterfaceParam, ( hDev, i, DEVICE_INFO_PERSISTENT_NETMASK_STRING, m_interfaceInfo[i].persistentSubnetMask_.c_str(), m_interfaceInfo[i].persistentSubnetMask_.length() ), WriteLogMessage )
			LOGGED_TLI_CALL( TLIMV_DevSetInterfaceParam, ( hDev, i, DEVICE_INFO_PERSISTENT_DEFAULT_GATEWAY_STRING, m_interfaceInfo[i].persistentDefaultGateway_.c_str(), m_interfaceInfo[i].persistentDefaultGateway_.length() ), WriteLogMessage )
		}
	}
	if( itDev->second->supportsUserDefinedName_ )
	{
		LOGGED_TLI_CALL( TLIMV_DevSetParam, ( hDev, DEVICE_INFO_USER_DEFINED_NAME, userDefinedName.mb_str(), userDefinedName.Length() ), WriteLogMessage )
	}

	WriteLogMessage( wxString::Format( wxT("Trying to close device %s(%s).\n"), serial.c_str(), product.c_str() ) );
	LOGGED_TLI_CALL( DevClose, ( hDev ), WriteLogMessage )
	WriteLogMessage( wxT("Done...\n") );
	UpdateDeviceList();
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::AssignTemporaryIP( int listItemIndex )
//-----------------------------------------------------------------------------
{
	wxString deviceMACAddress(wxEmptyString);
	wxString connectedIPAddress(wxEmptyString);
	if( listItemIndex >= 0 )
	{
		wxString itemText(m_pDevListCtrl->GetItemText( listItemIndex ));
		wxListItem info;
		info.m_itemId = listItemIndex;
		info.m_col = lcSerial;
		info.m_mask = wxLIST_MASK_TEXT;
		if( !m_pDevListCtrl->GetItem( info ) )
		{
			WriteLogMessage( wxString::Format( wxT("ERROR: Could not obtain serial number for selected device %s.\n"), itemText.c_str() ) );
		}

		DeviceMap::const_iterator itDev = m_devices.find( string(m_pSTSerialNumber->GetLabel().mb_str()) );
		if( itDev == m_devices.end() )
		{
			WriteLogMessage( wxString::Format( wxT("ERROR: Could not obtain device name for selected device %s on adapter %s.\n"), m_pSTSerialNumber->GetLabel().c_str(), m_pCBConnectedToIPAddress->GetValue().c_str() ) );
		}
		deviceMACAddress = ConvertedString(itDev->second->interfaceInfo_[0].MACAddress_);
		connectedIPAddress = m_pCBConnectedToIPAddress->GetValue();
	}
	AssignIPDlg dlg(this, m_hTLI, deviceMACAddress, connectedIPAddress);
	if( dlg.ShowModal() == wxID_OK )
	{
		UpdateDeviceList();
	}
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::BuildList( void )
//-----------------------------------------------------------------------------
{
	m_pDevListCtrl->DeleteAllItems();
	DeviceMap::const_iterator it = m_devices.begin();
	DeviceMap::const_iterator itEND = m_devices.end();
	long devCount = 0;
	while( it != itEND )
	{
		bool boNetmasksMatch = ( it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].second.netMask_ == it->second->interfaceInfo_[0].currentSubnetMask_ );
		if( !boNetmasksMatch )
		{
			WriteLogMessage( wxString::Format( wxT("WARNING: Device %s is not configured properly to be fully accessible at adapter %s (both the device and the network adapter it is connected to don't use the same netmask). Use 'Action -> Assign Temporary IPv4 Address' to set up the device correctly.\n"), ConvertedString(it->second->interfaceInfo_[0].currentIPAddress_).c_str(), ConvertedString(it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].first).c_str()), m_ERROR_STYLE );
		}

		bool boAddressesMatch = false;
		if( boNetmasksMatch )
		{
			if( m_pTLIMV_DoAdressesMatch )
			{
				boAddressesMatch = ( m_pTLIMV_DoAdressesMatch( it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].first.c_str(), it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].second.netMask_.c_str(), it->second->interfaceInfo_[0].currentIPAddress_.c_str(), it->second->interfaceInfo_[0].currentSubnetMask_.c_str() ) == 0 );
				if( !boAddressesMatch )
				{
					WriteLogMessage( wxString::Format( wxT("WARNING: Device %s is not configured properly to be fully accessible at adapter %s (both the device and the adapter it is connected to use the same netmask, but don't reside in the same net). Use 'Action -> Assign Temporary IPv4 Address' to set up the device correctly.\n"), ConvertedString(it->second->interfaceInfo_[0].currentIPAddress_).c_str(), ConvertedString(it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].first).c_str()), m_ERROR_STYLE );
				}
			}
		}
		long index = m_pDevListCtrl->InsertItem( devCount, ConvertedString(it->second->modelName_), ( boNetmasksMatch && boAddressesMatch ) ? 1 : 0 );
		m_pDevListCtrl->SetItem( index, lcSerial, ConvertedString(it->first) );
		m_pDevListCtrl->SetItem( index, lcPrimaryInterfaceIPAddress, ConvertedString(it->second->interfaceInfo_[0].currentIPAddress_) );
		m_pDevListCtrl->SetItemData( index, it->second->id_ );
		++it;
		++devCount;
	}

	m_pDevListCtrl->SetColumnWidth( lcProduct, ( ( devCount == 0 ) ? wxLIST_AUTOSIZE_USEHEADER : wxLIST_AUTOSIZE ) );
	m_pDevListCtrl->SetColumnWidth( lcSerial, ( ( devCount == 0 ) ? wxLIST_AUTOSIZE_USEHEADER : wxLIST_AUTOSIZE ) );
	m_pDevListCtrl->SetColumnWidth( lcPrimaryInterfaceIPAddress, wxLIST_AUTOSIZE_USEHEADER );
	UpdateDlgControls( false );
}

//-----------------------------------------------------------------------------
std::string IPConfigureFrame::GetInterfaceStringInfo( MVTLI_INTERFACE_HANDLE hInterface, INTERFACE_INFO_CMD info )
//-----------------------------------------------------------------------------
{
	if( !m_pIFGetInfo )
	{
		WriteLogMessage( wxT("IFGetInfo is not available.\n"), m_ERROR_STYLE );
		return string("");
	}

	size_t stringSize = 0;
	int result = m_pIFGetInfo( hInterface, info, 0, 0, &stringSize );
	if( result != 0 )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR during call to IFGetInfo( %p, %d, 0, 0, %p ): %d.\n"), hInterface, info, &stringSize, result ), m_ERROR_STYLE );
		return string("");
	}
	auto_array_ptr<char> pStringBuffer(stringSize);
	result = m_pIFGetInfo( hInterface, info, 0, pStringBuffer.get(), &stringSize );
	if( result != 0 )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR during call to IFGetInfo( %p, %d, 0, %p, %p ): %d.\n"), hInterface, info, pStringBuffer.get(), &stringSize, result ), m_ERROR_STYLE );
		return string("");
	}
	return string(pStringBuffer.get());
}

//-----------------------------------------------------------------------------
std::string IPConfigureFrame::GetDeviceInterfaceStringInfo( MVTLI_INTERFACE_HANDLE hInterface, const std::string& deviceName, unsigned int interfaceIndex, DEVICE_INFO_CMD info )
//-----------------------------------------------------------------------------
{
	if( !m_pTLIMV_IFGetDeviceInterfaceInfo )
	{
		WriteLogMessage( wxT("TLIMV_IFGetDeviceInterfaceInfo is not available.\n"), m_ERROR_STYLE );
		return string("");
	}

	size_t stringSize = 0;
	int result = m_pTLIMV_IFGetDeviceInterfaceInfo( hInterface, deviceName.c_str(), interfaceIndex, info, 0, 0, &stringSize );
	if( result != 0 )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR during call to TLIMV_IFGetDeviceInterfaceInfo( %p, %s, %d, %d, 0, 0, %p ): %d.\n"), hInterface, deviceName.c_str(), interfaceIndex, info, &stringSize, result ), m_ERROR_STYLE );
		return string("");
	}
	auto_array_ptr<char> pStringBuffer(stringSize);
	result = m_pTLIMV_IFGetDeviceInterfaceInfo( hInterface, deviceName.c_str(), interfaceIndex, info, 0, pStringBuffer.get(), &stringSize );
	if( result != 0 )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR during call to TLIMV_IFGetDeviceInterfaceInfo( %p, %s, %d, %d, 0, %p, %p ): %d.\n"), hInterface, deviceName.c_str(), interfaceIndex, info, pStringBuffer.get(), &stringSize, result ), m_ERROR_STYLE );
		return string("");
	}
	return string(pStringBuffer.get());
}

//-----------------------------------------------------------------------------
string IPConfigureFrame::GetDeviceStringInfo( MVTLI_INTERFACE_HANDLE hInterface, const string& deviceName, DEVICE_INFO_CMD info )
//-----------------------------------------------------------------------------
{
	if( !m_pIFGetDeviceInfo )
	{
		WriteLogMessage( wxT("IFGetDeviceInfo is not available.\n"), m_ERROR_STYLE );
		return string("");
	}

	size_t stringSize = 0;
	int result = m_pIFGetDeviceInfo( hInterface, deviceName.c_str(), info, 0, 0, &stringSize );
	if( result != 0 )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR during call to IFGetDeviceInfo( %p, %s, %d, 0, 0, %p ): %d.\n"), hInterface, deviceName.c_str(), info, &stringSize, result ), m_ERROR_STYLE );
		return string("");
	}
	auto_array_ptr<char> pStringBuffer(stringSize);
	result = m_pIFGetDeviceInfo( hInterface, deviceName.c_str(), info, 0, pStringBuffer.get(), &stringSize );
	if( result != 0 )
	{
		WriteLogMessage( wxString::Format( wxT("ERROR during call to IFGetDeviceInfo( %p, %s, %d, 0, %p, %p ): %d.\n"), hInterface, deviceName.c_str(), info, pStringBuffer.get(), &stringSize, result ), m_ERROR_STYLE );
		return string("");
	}
	return string(pStringBuffer.get());
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnAbout( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	wxBoxSizer *pTopDownSizer;
	wxDialog dlg(this, wxID_ANY, wxString(_("About mvIPConfigure")));
	wxIcon icon(mvIcon_xpm);
	dlg.SetIcon( icon );
	pTopDownSizer = new wxBoxSizer(wxVERTICAL);
	wxStaticText *pText = new wxStaticText( &dlg, wxID_ANY, wxT("Configuration tool for GigE Vision (tm) devices") );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("(C) 2008 - %s by %s"), CURRENT_YEAR, COMPANY_NAME ) );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("Version %s"), VERSION_STRING ) );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxT("Support contact: www.matrix-vision.de") );
	pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
	pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT("This tool has been written using wxWidgets (www.wxwidgets.org) and was compiled with version %d.%d.%d of this library"), wxMAJOR_VERSION, wxMINOR_VERSION, wxRELEASE_NUMBER ) );
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
void IPConfigureFrame::OnAssignTemporaryIP( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	AssignTemporaryIP( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnBtnConfigure( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	UpdateDlgControls( true );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnCBUseDHCP( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	m_interfaceInfo[m_pSCInterfaceSelector->GetValue()].DHCPEnabled_ = m_pCBUseDHCP->GetValue();
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnCBUsePersistentIP( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	m_interfaceInfo[m_pSCInterfaceSelector->GetValue()].persistentIPEnabled_ = m_pCBUsePersistentIP->GetValue();
	m_pTCPersistentIPAddress->Enable( m_pCBUsePersistentIP->GetValue() );
	m_pTCPersistentSubnetMask->Enable( m_pCBUsePersistentIP->GetValue() );
	m_pTCPersistentDefaultGateway->Enable( m_pCBUsePersistentIP->GetValue() );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnClose( wxCloseEvent& )
//-----------------------------------------------------------------------------
{
	Deinit();
	Destroy();
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnListItemDeselected( int /*listItemIndex*/ )
//-----------------------------------------------------------------------------
{
	UpdateDlgControls( false );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnListItemSelected( int /*listItemIndex*/ )
//-----------------------------------------------------------------------------
{
	m_pCBConnectedToIPAddress->Clear();
	UpdateDlgControls( false );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnConnectedToIPAddressTextChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	int currentItem = m_pDevListCtrl->GetCurrentItemIndex();
	wxListItem info;
	if( currentItem >= 0 )
	{
		info.m_itemId = currentItem;
		info.m_col = lcSerial;
		info.m_mask = wxLIST_MASK_TEXT;
		if( !m_pDevListCtrl->GetItem( info ) )
		{
			return;
		}
	}
	SetupNetworkGUIElements( m_devices.find( string(info.m_text.mb_str()) ), m_pSCInterfaceSelector->GetValue() );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnInterfaceSelectorTextChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	WriteLogMessage( wxString::Format(wxT("Interface %d selected\n"), m_pSCInterfaceSelector->GetValue() ) );
	UpdateDlgControls( true );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnPersistentIPTextChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	if( m_pTCPersistentIPAddress )
	{
		m_interfaceInfo[m_pSCInterfaceSelector->GetValue()].persistentIPAddress_ = m_pTCPersistentIPAddress->GetValue().mb_str();
	}
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnPersistentNetmaskTextChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	if( m_pTCPersistentSubnetMask )
	{
		m_interfaceInfo[m_pSCInterfaceSelector->GetValue()].persistentSubnetMask_ = m_pTCPersistentSubnetMask->GetValue().mb_str();
	}
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnPersistentGatewayTextChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	if( m_pTCPersistentDefaultGateway )
	{
		m_interfaceInfo[m_pSCInterfaceSelector->GetValue()].persistentDefaultGateway_ = m_pTCPersistentDefaultGateway->GetValue().mb_str();
	}
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnQuit( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	// true is to force the frame to close
	Close( true );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnTimer( wxTimerEvent& e )
//-----------------------------------------------------------------------------
{
	switch( e.GetId() )
	{
	case teQuit:
		Close( true );
		break;
	default:
		break;
	}
	e.Skip();
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnUpdateDeviceList( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	UpdateDeviceList();
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::OnUseAdvancedDeviceDiscovery( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	UpdateDeviceList();
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::SetupNetworkGUIElements( DeviceMap::const_iterator it, const int interfaceIndex )
//-----------------------------------------------------------------------------
{
	bool boDeviceValid = ( it != m_devices.end() );
	bool boMarkNetmaskConflict = false;
	bool boMarkIPAddressConflict = false;

	if( boDeviceValid )
	{
		m_pSTConnectedToNetmask->SetLabel( ConvertedString(it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].second.netMask_.c_str()) );
		m_pSTConnectedToMTU->SetLabel( wxString::Format( wxT("%d"), it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].second.MTU_ ) );
		m_pSTConnectedToLinkSpeed->SetLabel( wxString::Format( wxT("%d"), it->second->adapters_[m_pCBConnectedToIPAddress->GetSelection()].second.linkSpeed_ ) );
		if( interfaceIndex == 0 )
		{
			boMarkNetmaskConflict = ( string(m_pSTConnectedToNetmask->GetLabel().mb_str()) != it->second->interfaceInfo_[0].currentSubnetMask_ );
			boMarkIPAddressConflict = m_pTLIMV_DoAdressesMatch ? ( m_pTLIMV_DoAdressesMatch( m_pCBConnectedToIPAddress->GetValue().mb_str(), m_pSTConnectedToNetmask->GetLabel().mb_str(), it->second->interfaceInfo_[0].currentIPAddress_.c_str(), it->second->interfaceInfo_[0].currentSubnetMask_.c_str() ) != 0 ) : true;
		}
	}
	else
	{
		m_pSTConnectedToNetmask->SetLabel( wxT("-") );
		m_pSTConnectedToMTU->SetLabel( wxT("-") );
		m_pSTConnectedToLinkSpeed->SetLabel( wxT("-") );
	}

	wxColour defaultColour(m_pSTMACAddress->GetBackgroundColour());
	m_pSTCurrentIPAddress->SetBackgroundColour( boMarkIPAddressConflict ? m_ERROR_STYLE.GetTextColour() : defaultColour );
	m_pSTCurrentSubnetMask->SetBackgroundColour( boMarkNetmaskConflict ? m_ERROR_STYLE.GetTextColour() : defaultColour );
	m_pCBConnectedToIPAddress->SetBackgroundColour( boMarkIPAddressConflict ? m_ERROR_STYLE.GetTextColour() : *wxWHITE );
	m_pSTConnectedToNetmask->SetBackgroundColour( boMarkNetmaskConflict ? m_ERROR_STYLE.GetTextColour() : defaultColour );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::UpdateDeviceList( void )
//-----------------------------------------------------------------------------
{
	wxBusyCursor busyCursorScope;
	WriteLogMessage( wxString(wxT("Updating device list...\n")) );
	int status = 0;
	char hasChanged = 0;
	LOGGED_TLI_CALL( TLUpdateInterfaceList, ( m_hTLI, &hasChanged, 0 ), WriteLogMessage );
	unsigned int interfaceCnt = 0;
	LOGGED_TLI_CALL( TLGetNumInterfaces, ( m_hTLI, &interfaceCnt ), WriteLogMessage );
	if( status != 0 )
	{
		return;
	}

	if( interfaceCnt < 1 )
	{
		WriteLogMessage( wxT("No interfaces detected.\n"), m_ERROR_STYLE );
		return;
	}

	WriteLogMessage( wxString::Format( wxT("%d interface%s detected.\n"), interfaceCnt, ( interfaceCnt > 1 ) ? wxT("s") : wxT("") ) );

	InterfaceContainer lastInterfaceList(m_TLIInterfaces); // after this loop, this list will contain all the interfaces, that have disappeared...
	for( unsigned int i=0; i<interfaceCnt; i++ )
	{
		size_t stringSize = 0;
		LOGGED_TLI_CALL_WITH_CONTINUE( TLGetInterfaceID, ( m_hTLI, i, 0, &stringSize ), WriteLogMessage )
		auto_array_ptr<char> pStringBuffer(stringSize);
		LOGGED_TLI_CALL_WITH_CONTINUE( TLGetInterfaceID, ( m_hTLI, i, pStringBuffer.get(), &stringSize ), WriteLogMessage )
		if( m_TLIInterfaces.find( string(pStringBuffer.get() ) ) == m_TLIInterfaces.end() )
		{
			// this is a new interface
			MVTLI_INTERFACE_HANDLE hInterface = 0;
			LOGGED_TLI_CALL_WITH_CONTINUE( TLOpenInterface, ( m_hTLI, pStringBuffer.get(), &hInterface ), WriteLogMessage )
			m_TLIInterfaces.insert( make_pair( string(pStringBuffer.get()), hInterface ) );
		}
		else
		{
			// this interface should still be there
			InterfaceContainer::iterator it = lastInterfaceList.find( string(pStringBuffer.get() ) );
			assert( ( it != lastInterfaceList.end() ) && "BUG detected in interface handling. If this interface is missing in the list of interfaces detected last time there is a bug in the implementation" );
			if( it != lastInterfaceList.end() )
			{
				lastInterfaceList.erase( it );
			}
		}
	}
	InterfaceContainer::iterator itInterface = m_TLIInterfaces.begin();
	InterfaceContainer::iterator itInterfaceEND = m_TLIInterfaces.end();
	while( itInterface != itInterfaceEND )
	{
		InterfaceContainer::iterator itLast = lastInterfaceList.find( itInterface->first );
		if( itLast != lastInterfaceList.end() )
		{
			// this interface is gone now...
			/// \todo close it?
			m_TLIInterfaces.erase( itInterface );
			itInterface = m_TLIInterfaces.begin();
		}
		else
		{
			++itInterface;
		}
	}

	for_each( m_devices.begin(), m_devices.end(), ptr_fun(DeleteSecond<const string, DetecedDeviceInfo*>) );
	m_devices.clear();
	InterfaceContainer::iterator itInterfaces = m_TLIInterfaces.begin();
	InterfaceContainer::iterator itInterfacesEnd = m_TLIInterfaces.end();
	while( itInterfaces != itInterfacesEnd )
	{
		unsigned int deviceDiscoveryMode = m_pMISettings_UseAdvancedDeviceDiscovery->IsChecked() ? 1 : 0;
		size_t deviceDiscoveryModeSize = sizeof(deviceDiscoveryMode);
		LOGGED_TLI_CALL( TLIMV_IFSetInterfaceParam, ( itInterfaces->second, INTERFACE_INFO_ADVANCED_DEVICE_DISCOVERY_MODE, 0, &deviceDiscoveryMode, deviceDiscoveryModeSize ), WriteLogMessage );
		// each device is supposed to answer within 1 second
		char hasChanged = 0;
		LOGGED_TLI_CALL( IFUpdateDeviceList, ( itInterfaces->second, &hasChanged, 1100 ), WriteLogMessage )
		unsigned int deviceCnt = 0;
		if( m_pIFGetNumDevices )
		{
			status = m_pIFGetNumDevices( itInterfaces->second, &deviceCnt );
			if( status == 0 )
			{
				WriteLogMessage( wxString::Format( wxT("Interface %s reported %d device%s.\n"), ConvertedString( itInterfaces->first ).c_str(), deviceCnt, ( deviceCnt != 1 ) ? wxT("s") : wxT("") ) );
				if( m_pIFGetDeviceID && m_pIFGetDeviceInfo && m_pTLIMV_IFGetDeviceInterfaceInfo )
				{
					for( unsigned int i=0; i<deviceCnt; i++ )
					{
						size_t stringSize = 0;
						LOGGED_TLI_CALL_WITH_CONTINUE( IFGetDeviceID, ( itInterfaces->second, i, 0, &stringSize ), WriteLogMessage )
						auto_array_ptr<char> pStringBuffer(stringSize);
						LOGGED_TLI_CALL_WITH_CONTINUE( IFGetDeviceID, ( itInterfaces->second, i, pStringBuffer.get(), &stringSize ), WriteLogMessage )

						string deviceName(pStringBuffer.get());
						string serial = GetDeviceStringInfo( itInterfaces->second, deviceName, DEVICE_INFO_SERIALNUMBER );
						DeviceMap::iterator itDev = m_devices.find( serial );

						unsigned int adapterMTU = numeric_limits<unsigned int>::max();
						size_t bufferSize = sizeof(adapterMTU);
						LOGGED_TLI_CALL( IFGetInfo, ( itInterfaces->second, INTERFACE_INFO_MTU, 0, &adapterMTU, &bufferSize ), WriteLogMessage )
						unsigned int adapterLinkSpeed = 0;
						bufferSize = sizeof(adapterLinkSpeed);
						LOGGED_TLI_CALL( IFGetInfo, ( itInterfaces->second, INTERFACE_INFO_LINK_SPEED, 0, &adapterLinkSpeed, &bufferSize ), WriteLogMessage )
						const string adapterIPAddress(GetInterfaceStringInfo( itInterfaces->second, INTERFACE_INFO_IP_STRING ));
						const string adapterNetmask(GetInterfaceStringInfo( itInterfaces->second, INTERFACE_INFO_NETMASK_STRING ));
						if( itDev == m_devices.end() )
						{
							// this is the first time this device has been located in this 'Enumerate' run. It might 
							// however be found again at a different network adapter
							const wxString GEVType(wxT("GEV"));
							const wxString TLType(ConvertedString(GetDeviceStringInfo( itInterfaces->second, deviceName, DEVICE_INFO_TLTYPE )));
							if( TLType != GEVType )
							{
								WriteLogMessage( wxString::Format( wxT("Device %s reports its transport layer technology as %s while this application supports GEV devices only. This device therefore will be ignored here.\n"), ConvertedString(serial).c_str(), TLType.c_str() ), wxTextAttr(wxColour(0, 0, 255)) );
								continue;
							}
							string model(GetDeviceStringInfo( itInterfaces->second, deviceName, DEVICE_INFO_MODEL ));
							string manufacturer(GetDeviceStringInfo( itInterfaces->second, deviceName, DEVICE_INFO_VENDOR ));
							string userDefinedName(GetDeviceStringInfo( itInterfaces->second, deviceName, DEVICE_INFO_USER_DEFINED_NAME ));
							unsigned int interfaceCount = 1;
							bufferSize = sizeof(interfaceCount);
							LOGGED_TLI_CALL( IFGetDeviceInfo, ( itInterfaces->second, deviceName.c_str(), DEVICE_INFO_INTERFACE_COUNT, 0, &interfaceCount, &bufferSize ), WriteLogMessage )
							if( interfaceCount > DetecedDeviceInfo::MAX_INTERFACE_COUNT )
							{
								WriteLogMessage( wxString::Format( wxT("ERROR!!! This device claims to support %u interfaces, while the current version of the standard only allows %lu interfaces.\n"), interfaceCount, (long unsigned int) DetecedDeviceInfo::MAX_INTERFACE_COUNT ), m_ERROR_STYLE );
								interfaceCount = 4;
							}
							unsigned char supportsUserDefinedName = 0;
							bufferSize = sizeof(supportsUserDefinedName);
							LOGGED_TLI_CALL( IFGetDeviceInfo, ( itInterfaces->second, deviceName.c_str(), DEVICE_INFO_SUPPORTS_USER_DEFINED_NAME, 0, &supportsUserDefinedName, &bufferSize ), WriteLogMessage )
							DetecedDeviceInfo* p = new DetecedDeviceInfo(deviceName, serial, model, manufacturer, userDefinedName, supportsUserDefinedName, adapterIPAddress, adapterNetmask, adapterMTU, adapterLinkSpeed, interfaceCount, static_cast<long>(m_devices.size()));
							pair<DeviceMap::iterator, bool> insertPair = m_devices.insert( make_pair( serial, p ) );
							if( !insertPair.second )
							{
								WriteLogMessage( wxT("Internal ERROR!!! This device has been inserted already.\n"), m_ERROR_STYLE );
							}
							else
							{
								WriteLogMessage( wxString::Format( wxT("Device %s detected on interface %s).\n"), ConvertedString(serial).c_str(), ConvertedString(adapterIPAddress).c_str() ) );
								for( unsigned int i=0; i<interfaceCount; i++ )
								{
									InterfaceInfo& info = insertPair.first->second->interfaceInfo_[i];
									info.currentIPAddress_ = GetDeviceInterfaceStringInfo( itInterfaces->second, deviceName, i, DEVICE_INFO_IP_STRING );
									info.currentSubnetMask_ = GetDeviceInterfaceStringInfo( itInterfaces->second, deviceName, i, DEVICE_INFO_CURRENT_NETMASK_STRING );
									info.currentDefaultGateway_ = GetDeviceInterfaceStringInfo( itInterfaces->second, deviceName, i, DEVICE_INFO_CURRENT_DEFAULT_GATEWAY_STRING );
									info.persistentIPAddress_ = GetDeviceInterfaceStringInfo( itInterfaces->second, deviceName, i, DEVICE_INFO_PERSISTENT_IP_STRING );
									info.persistentSubnetMask_ = GetDeviceInterfaceStringInfo( itInterfaces->second, deviceName, i, DEVICE_INFO_PERSISTENT_NETMASK_STRING );
									info.persistentDefaultGateway_ = GetDeviceInterfaceStringInfo( itInterfaces->second, deviceName, i, DEVICE_INFO_PERSISTENT_DEFAULT_GATEWAY_STRING );
									info.MACAddress_ = GetDeviceInterfaceStringInfo( itInterfaces->second, deviceName, i, DEVICE_INFO_MAC_STRING );
									bufferSize = sizeof(info.supportsLLA_);
									LOGGED_TLI_CALL( TLIMV_IFGetDeviceInterfaceInfo, ( itInterfaces->second, deviceName.c_str(), i, DEVICE_INFO_SUPPORTS_IP_LLA, 0, &info.supportsLLA_, &bufferSize ), WriteLogMessage )
									bufferSize = sizeof(info.supportsDHCP_);
									LOGGED_TLI_CALL( TLIMV_IFGetDeviceInterfaceInfo, ( itInterfaces->second, deviceName.c_str(), i, DEVICE_INFO_SUPPORTS_IP_DHCP, 0, &info.supportsDHCP_, &bufferSize ), WriteLogMessage )
									bufferSize = sizeof(info.supportsPersistentIP_);
									LOGGED_TLI_CALL( TLIMV_IFGetDeviceInterfaceInfo, ( itInterfaces->second, deviceName.c_str(), i, DEVICE_INFO_SUPPORTS_IP_PERSISTENT, 0, &info.supportsPersistentIP_, &bufferSize ), WriteLogMessage )
									bufferSize = sizeof(info.LLAEnabled_);
									LOGGED_TLI_CALL( TLIMV_IFGetDeviceInterfaceInfo, ( itInterfaces->second, deviceName.c_str(), i, DEVICE_INFO_CURRENT_IP_LLA, 0, &info.LLAEnabled_, &bufferSize ), WriteLogMessage )
									bufferSize = sizeof(info.DHCPEnabled_);
									LOGGED_TLI_CALL( TLIMV_IFGetDeviceInterfaceInfo, ( itInterfaces->second, deviceName.c_str(), i, DEVICE_INFO_CURRENT_IP_DHCP, 0, &info.DHCPEnabled_, &bufferSize ), WriteLogMessage )
									bufferSize = sizeof(info.persistentIPEnabled_);
									LOGGED_TLI_CALL( TLIMV_IFGetDeviceInterfaceInfo, ( itInterfaces->second, deviceName.c_str(), i, DEVICE_INFO_CURRENT_IP_PERSISTENT, 0, &info.persistentIPEnabled_, &bufferSize ), WriteLogMessage )
								}
							}
						}
						else
						{
							WriteLogMessage( wxString::Format( wxT("Device %s (first detected at interface %s) can also be reached via interface %s(%s).\n"), ConvertedString(serial).c_str(), ConvertedString(itDev->second->adapters_[0].first).c_str(), ConvertedString(adapterIPAddress).c_str(), ConvertedString(itInterfaces->first).c_str() ) );
							itDev->second->adapters_.push_back( make_pair( adapterIPAddress, AdapterInfo(adapterNetmask, adapterMTU, adapterLinkSpeed) ) );
						}
					}
				}
				else
				{
					WriteLogMessage( wxString::Format( wxT("At least one function pointer needed for the enumerate run could not be resolved. IFGetDeviceID: %p, IFGetDeviceInfo: %p, TLIMV_IFGetDeviceInterfaceInfo: %p.\n"), m_pIFGetDeviceID, m_pIFGetDeviceInfo, m_pTLIMV_IFGetDeviceInterfaceInfo ), m_ERROR_STYLE );
				}
			}
			else
			{
				WriteLogMessage( wxString::Format( wxT("ERROR during call to IFGetNumDevices( %p, %p ).\n"), itInterfaces->second, &deviceCnt ), m_ERROR_STYLE );
			}
		}
		else
		{
			WriteLogMessage( wxT("Pointer to IFGetNumDevices is invalid.\n"), m_ERROR_STYLE );
		}
		++itInterfaces;
	}
	BuildList();
}

//-----------------------------------------------------------------------------
int IPConfigureFrame::IsValidIPv4Address( const char* pData )
//-----------------------------------------------------------------------------
{
	int status = 0;
	CHECKED_TLI_CALL_WITH_RETURN( TLIMV_IsValidIPv4Address, ( pData ), WriteLogMessage )
}

//-----------------------------------------------------------------------------
int IPConfigureFrame::MACFromSerial( const char* pSerial, char* pBuf, size_t* pBufSize )
//-----------------------------------------------------------------------------
{
	int status = 0;
	CHECKED_TLI_CALL_WITH_RETURN( TLIMV_MACFromSerial, ( pSerial, pBuf, pBufSize ), WriteLogMessage )
}

//-----------------------------------------------------------------------------
int IPConfigureFrame::ForceIP( const char* pMACAddress, const char* pNewDeviceIPAddress, const char* pStaticSubnetMask, const char* pStaticDefaultGateway, const char* pAdapterIPAddress, unsigned int timeout_ms )
//-----------------------------------------------------------------------------
{
	int status = 0;
	CHECKED_TLI_CALL_WITH_RETURN( TLIMV_ForceIP, ( pMACAddress, pNewDeviceIPAddress, pStaticSubnetMask, pStaticDefaultGateway, pAdapterIPAddress, timeout_ms ), WriteLogMessage )
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::UpdateDlgControls( bool boEdit )
//-----------------------------------------------------------------------------
{
	int currentItem = m_pDevListCtrl->GetCurrentItemIndex();
	wxListItem info;
	if( currentItem >= 0 )
	{
		info.m_itemId = currentItem;
		info.m_col = lcSerial;
		info.m_mask = wxLIST_MASK_TEXT;
		if( !m_pDevListCtrl->GetItem( info ) )
		{
			return;
		}
	}

	DeviceMap::const_iterator it = m_devices.find( string(info.m_text.mb_str()) );
	bool boDeviceValid = ( it != m_devices.end() );
	if( boDeviceValid )
	{
		for( unsigned int i=0; i<DetecedDeviceInfo::MAX_INTERFACE_COUNT; i++ )
		{
			m_interfaceInfo[0] = it->second->interfaceInfo_[0];
		}
	}
	else
	{
		boEdit = false;
	}

	// device info controls
	m_pSTManufacturer->SetLabel( ConvertedString(boDeviceValid ? it->second->manufacturer_.c_str() : "-") );
	m_pSTSerialNumber->SetLabel( ConvertedString(boDeviceValid ? it->second->deviceSerial_.c_str() : "-") );
	m_pTCUserDefinedName->SetValue( ConvertedString(boDeviceValid ? it->second->userDefinedName_.c_str() : "-") );
	m_pTCUserDefinedName->Enable( boEdit && it->second->supportsUserDefinedName_ );
	m_pSTInterfaceCount->SetLabel( boDeviceValid ? wxString::Format( wxT("%d"), it->second->interfaceCount_ ) : wxT("-") );

	// interface selector
	m_pSCInterfaceSelector->Enable( boEdit );
	m_pSCInterfaceSelector->SetRange( 0, ( boDeviceValid ? it->second->interfaceCount_ - 1 : 0 ) );
	if( m_pSCInterfaceSelector->GetValue() > m_pSCInterfaceSelector->GetMax() )
	{
		m_pSCInterfaceSelector->SetValue( 0 );
	}

	const int interfaceIndex = m_pSCInterfaceSelector->GetValue();

	// current IP controls
	m_pSTCurrentIPAddress->SetLabel( ConvertedString(boDeviceValid ? m_interfaceInfo[interfaceIndex].currentIPAddress_.c_str() : "-") );
	m_pSTCurrentSubnetMask->SetLabel( ConvertedString(boDeviceValid ? m_interfaceInfo[interfaceIndex].currentSubnetMask_.c_str() : "-") );
	m_pSTCurrentDefaultGateway->SetLabel( ConvertedString(boDeviceValid ? m_interfaceInfo[interfaceIndex].currentDefaultGateway_.c_str() : "-") );
	m_pSTMACAddress->SetLabel( ConvertedString(boDeviceValid ? m_interfaceInfo[interfaceIndex].MACAddress_.c_str() : "-") );
	if( !boDeviceValid )
	{
		m_pCBConnectedToIPAddress->Clear();
		m_pCBConnectedToIPAddress->Append( wxT("-") );
		m_pCBConnectedToIPAddress->Select( 0 );
	}
	else if( m_pCBConnectedToIPAddress->IsEmpty() )
	{
		const std::vector<std::pair<std::string, std::string> >::size_type adapterCnt = it->second->adapters_.size();
		for( std::vector<std::pair<std::string, std::string> >::size_type i=0; i<adapterCnt; i++ )
		{
			m_pCBConnectedToIPAddress->Append( ConvertedString(it->second->adapters_[i].first.c_str()) );
		}
		m_pCBConnectedToIPAddress->Select( 0 );
	}
	m_pCBConnectedToIPAddress->Enable( boDeviceValid );
	SetupNetworkGUIElements( it, interfaceIndex );

	// persistent IP controls
	m_pTCPersistentIPAddress->SetValue( ConvertedString((boDeviceValid && m_interfaceInfo[interfaceIndex].supportsPersistentIP_) ? m_interfaceInfo[interfaceIndex].persistentIPAddress_.c_str() : "-") );
	m_pTCPersistentIPAddress->Enable( boEdit && m_interfaceInfo[interfaceIndex].supportsPersistentIP_ && m_interfaceInfo[interfaceIndex].persistentIPEnabled_ );
	m_pTCPersistentSubnetMask->SetValue( ConvertedString((boDeviceValid && m_interfaceInfo[interfaceIndex].supportsPersistentIP_) ? m_interfaceInfo[interfaceIndex].persistentSubnetMask_.c_str() : "-") );
	m_pTCPersistentSubnetMask->Enable( boEdit && m_interfaceInfo[interfaceIndex].supportsPersistentIP_ && m_interfaceInfo[interfaceIndex].persistentIPEnabled_ );
	m_pTCPersistentDefaultGateway->SetValue( ConvertedString((boDeviceValid && m_interfaceInfo[interfaceIndex].supportsPersistentIP_) ? m_interfaceInfo[interfaceIndex].persistentDefaultGateway_.c_str() : "-") );
	m_pTCPersistentDefaultGateway->Enable( boEdit && m_interfaceInfo[interfaceIndex].supportsPersistentIP_ && m_interfaceInfo[interfaceIndex].persistentIPEnabled_ );

	// IP configuration controls
	m_pCBUsePersistentIP->Enable( boEdit && m_interfaceInfo[interfaceIndex].supportsPersistentIP_ );
	m_pCBUsePersistentIP->SetValue( boDeviceValid && m_interfaceInfo[interfaceIndex].persistentIPEnabled_ );
	m_pCBUseDHCP->Enable( boEdit && m_interfaceInfo[interfaceIndex].supportsDHCP_ );
	m_pCBUseDHCP->SetValue( boDeviceValid && m_interfaceInfo[interfaceIndex].DHCPEnabled_ );
	// never enable this check box as this feature must always be active anyway
	m_pCBUseLLA->SetValue( boDeviceValid && m_interfaceInfo[interfaceIndex].LLAEnabled_ );

	// buttons
	m_pBtnConfigure->Enable( boDeviceValid && !boEdit );
	m_pBtnApplyChanges->Enable( boEdit );
}

//-----------------------------------------------------------------------------
void IPConfigureFrame::WriteLogMessage( const wxString& msg, const wxTextAttr& style /* = wxTextAttr(wxColour(0, 0, 0)) */ )
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
