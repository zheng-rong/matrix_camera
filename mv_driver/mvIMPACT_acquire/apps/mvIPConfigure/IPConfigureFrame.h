//-----------------------------------------------------------------------------
#ifndef IPConfigureFrameH
#define IPConfigureFrameH IPConfigureFrameH
//-----------------------------------------------------------------------------
#include "wx/wx.h"
#include "wx/dynlib.h"
#include "wx/imaglist.h"
#include <apps/Common/wxAbstraction.h>
#include "CustomValidators.h"
#include <map>
#include <string>
#include "TLILibImports.h"
#include <vector>

#define LOGGED_TLI_CALL(FN, PARAMS, LOGGER_CALL) \
	{ \
		if( m_p##FN ) \
		{ \
			status = m_p##FN PARAMS; \
			if( status != 0 ) \
			{ \
				string functionName(__FUNCTION__); \
				string FNString(#FN); \
				string PARAMSString(#PARAMS); \
				LOGGER_CALL( wxString::Format( wxT("%s: ERROR while calling %s%s: %d.\n"), ConvertedString(functionName).c_str(), ConvertedString(FNString).c_str(), ConvertedString(PARAMSString).c_str(), status ), wxColour(255, 0, 0) ); \
			} \
		} \
		else \
		{ \
			string functionName(__FUNCTION__); \
			string FNString(#FN); \
			string PARAMSString(#PARAMS); \
			LOGGER_CALL( wxString::Format( wxT("%s: Exported symbol '%s' could not be resolved/extracted from GenTL producer. Cannot call %s%s: %d.\n"), ConvertedString(functionName).c_str(), ConvertedString(FNString).c_str(), ConvertedString(FNString).c_str(), ConvertedString(PARAMSString).c_str(), status ), wxColour(255, 0, 0) ); \
		} \
	}

#define LOGGED_TLI_CALL_WITH_CONTINUE(FN, PARAMS, LOGGER_CALL) \
	{ \
		if( m_p##FN ) \
		{ \
			status = m_p##FN PARAMS; \
			if( status != 0 ) \
			{ \
				string functionName(__FUNCTION__); \
				string FNString(#FN); \
				string PARAMSString(#PARAMS); \
				LOGGER_CALL( wxString::Format( wxT("%s: ERROR while calling %s%s: %d.\n"), ConvertedString(functionName).c_str(), ConvertedString(FNString).c_str(), ConvertedString(PARAMSString).c_str(), status ), wxColour(255, 0, 0) ); \
				continue; \
			} \
		} \
		else \
		{ \
			string functionName(__FUNCTION__); \
			string FNString(#FN); \
			string PARAMSString(#PARAMS); \
			LOGGER_CALL( wxString::Format( wxT("%s: Exported symbol '%s' could not be resolved/extracted from GenTL producer. Cannot call %s%s: %d.\n"), ConvertedString(functionName).c_str(), ConvertedString(FNString).c_str(), ConvertedString(FNString).c_str(), ConvertedString(PARAMSString).c_str(), status ), wxColour(255, 0, 0) ); \
		} \
	}

#define CHECKED_TLI_CALL_WITH_RETURN(FN, PARAMS, LOGGER_CALL) \
	{ \
		if( m_p##FN ) \
		{ \
			status = m_p##FN PARAMS; \
		} \
		else \
		{ \
			string functionName(__FUNCTION__); \
			string FNString(#FN); \
			string PARAMSString(#PARAMS); \
			LOGGER_CALL( wxString::Format( wxT("%s: Exported symbol '%s' could not be resolved/extracted from GenTL producer. Cannot call %s%s: %d.\n"), ConvertedString(functionName).c_str(), ConvertedString(FNString).c_str(), ConvertedString(FNString).c_str(), ConvertedString(PARAMSString).c_str(), status ), wxColour(255, 0, 0) ); \
		} \
		return status; \
	}

#define EXTRACT_SYMBOL(FN, LOGGER_CALL) \
	m_p##FN = (P##FN)m_TLILib.GetSymbol( ConvertedString(#FN) ); \
	if( !m_p##FN ) \
	{ \
		string functionName(__FUNCTION__); \
		string FNString(#FN); \
		LOGGER_CALL( wxString::Format( wxT("%s: Exported symbol '%s' could not be resolved/extracted from GenTL producer.\n"), ConvertedString(functionName).c_str(), ConvertedString(FNString).c_str() ), wxColour(255, 0, 0) ); \
	}

//-----------------------------------------------------------------------------
struct InterfaceInfo
//-----------------------------------------------------------------------------
{
	explicit InterfaceInfo() : currentIPAddress_(), currentSubnetMask_(), currentDefaultGateway_(), persistentIPAddress_(),
		persistentSubnetMask_(), persistentDefaultGateway_(), MACAddress_(), supportsLLA_(0), supportsDHCP_(0), supportsPersistentIP_(0),
		LLAEnabled_(1), DHCPEnabled_(0), persistentIPEnabled_(0) {}
	std::string currentIPAddress_;
	std::string currentSubnetMask_;
	std::string currentDefaultGateway_;
	std::string persistentIPAddress_;
	std::string persistentSubnetMask_;
	std::string persistentDefaultGateway_;
	std::string MACAddress_;
	unsigned char supportsLLA_;
	unsigned char supportsDHCP_;
	unsigned char supportsPersistentIP_;
	unsigned char LLAEnabled_;
	unsigned char DHCPEnabled_;
	unsigned char persistentIPEnabled_;
};

//-----------------------------------------------------------------------------
struct AdapterInfo
//-----------------------------------------------------------------------------
{
	std::string netMask_;
	unsigned int MTU_;
	unsigned int linkSpeed_;
	explicit AdapterInfo( const std::string& netMask, unsigned int MTU, unsigned int linkSpeed ) : netMask_(netMask), MTU_(MTU), linkSpeed_(linkSpeed) {}
};

//-----------------------------------------------------------------------------
struct DetecedDeviceInfo
//-----------------------------------------------------------------------------
{
	std::string deviceName_;
	std::string deviceSerial_;
	std::string modelName_;
	std::string manufacturer_;
	std::string userDefinedName_;
	unsigned char supportsUserDefinedName_;
	std::vector<std::pair<std::string, AdapterInfo> > adapters_;
	unsigned int interfaceCount_;
	static const size_t MAX_INTERFACE_COUNT = 4;
	InterfaceInfo interfaceInfo_[MAX_INTERFACE_COUNT];
	long id_;
	DetecedDeviceInfo( const std::string& deviceName, const std::string& devSerial, const std::string& modelName,
		const std::string& manufacturer, const std::string& userDefinedName, unsigned char const supportsUserDefinedName,
		std::string adapterIPAddress, const std::string& adapterNetmask, unsigned int adapterMTU, unsigned int adapterLinkSpeed, unsigned int interfaceCount,
		long id )
		: deviceName_(deviceName), deviceSerial_(devSerial), modelName_(modelName), manufacturer_(manufacturer), userDefinedName_(userDefinedName),
		supportsUserDefinedName_(supportsUserDefinedName), interfaceCount_(interfaceCount), id_(id)
	{
		adapters_.push_back( std::make_pair( adapterIPAddress, AdapterInfo(adapterNetmask, adapterMTU, adapterLinkSpeed) ) );
	}
};

class DeviceListCtrl;
class wxSplitterWindow;
class wxSpinCtrl;

typedef std::map<std::string, MVTLI_INTERFACE_HANDLE> InterfaceContainer;
typedef std::map<std::string, DetecedDeviceInfo*> DeviceMap;

//-----------------------------------------------------------------------------
class IPConfigureFrame : public wxFrame
//-----------------------------------------------------------------------------
{
public:
	explicit					IPConfigureFrame( const wxString& title, const wxPoint& pos, const wxSize& size, int argc, wxChar** argv );
							   ~IPConfigureFrame();
	void						AssignTemporaryIP( int listItemIndex );
	const DeviceMap&			GetDeviceMap( void ) const { return m_devices; }
	std::string					GetInterfaceStringInfo( MVTLI_INTERFACE_HANDLE hInterface, INTERFACE_INFO_CMD info );
	std::string					GetDeviceInterfaceStringInfo( MVTLI_INTERFACE_HANDLE hInterface, const std::string& deviceName, unsigned int interfaceIndex, DEVICE_INFO_CMD info );
	std::string					GetDeviceStringInfo( MVTLI_INTERFACE_HANDLE hInterface, const std::string& deviceName, DEVICE_INFO_CMD info );
	const InterfaceContainer&	GetInterfaces( void ) const { return m_TLIInterfaces; }
	void						OnListItemSelected( int listItemIndex );
	void						OnListItemDeselected( int listItemIndex );
	void						WriteLogMessage( const wxString& msg, const wxTextAttr& style = wxTextAttr(wxColour(0, 0, 0)) );

	int							IsValidIPv4Address( const char* pData );
	int							MACFromSerial( const char* pSerial, char* pBuf, size_t* pBufSize );
	int							ForceIP( const char* pMACAddress, const char* pNewDeviceIPAddress, const char* pStaticSubnetMask, const char* pStaticDefaultGateway, const char* pAdapterIPAddress, unsigned int timeout_ms );
protected:
	// event handlers (these functions should _not_ be virtual)
	void OnAbout( wxCommandEvent& e );
	void OnAssignTemporaryIP( wxCommandEvent& e );
	void OnBtnApplyChanges( wxCommandEvent& e );
	void OnBtnConfigure( wxCommandEvent& e );
	void OnCBUseDHCP( wxCommandEvent& e );
	void OnCBUsePersistentIP( wxCommandEvent& e );
	void OnClose( wxCloseEvent& e );
	void OnConnectedToIPAddressTextChanged( wxCommandEvent& e );
	void OnInterfaceSelectorTextChanged( wxCommandEvent& e );
	void OnPersistentIPTextChanged( wxCommandEvent& e );
	void OnPersistentNetmaskTextChanged( wxCommandEvent& e );
	void OnPersistentGatewayTextChanged( wxCommandEvent& e );
	void OnQuit( wxCommandEvent& e );
	void OnTimer( wxTimerEvent& e );
	void OnUpdateDeviceList( wxCommandEvent& e );
	void OnUseAdvancedDeviceDiscovery( wxCommandEvent& e );
private:
	//-----------------------------------------------------------------------------
	enum TTimerEvent
	//-----------------------------------------------------------------------------
	{
		teQuit
	};
	void								ApplyChanges( const wxString& serial, const wxString& product, const wxString& connectedToIPAddress, const wxString& userDefinedName );
	void								BuildList( void );
	void								Deinit( void );
	bool								SelectDevice( const wxString& deviceToConfigure );
	void								SetupNetworkGUIElements( DeviceMap::const_iterator it, const int interfaceIndex );
	void								UpdateDeviceList( void );
	void								UpdateDlgControls( bool boEdit );
	DeviceListCtrl*						m_pDevListCtrl;
	wxTimer								m_quitTimer;
	wxButton*							m_pBtnConfigure;
	wxButton*							m_pBtnApplyChanges;
	wxCheckBox*							m_pCBUsePersistentIP;
	wxCheckBox*							m_pCBUseDHCP;
	wxCheckBox*							m_pCBUseLLA;
	wxMenuItem*							m_pMISettings_UseAdvancedDeviceDiscovery;
	wxTextCtrl*							m_pLogWindow;
	wxSpinCtrl*							m_pSCInterfaceSelector;
	wxStaticText*						m_pSTManufacturer;
	wxStaticText*						m_pSTSerialNumber;
	wxStaticText*						m_pSTInterfaceCount;
	wxStaticText*						m_pSTMACAddress;
	wxTextCtrl*							m_pTCUserDefinedName;
	wxStaticText*						m_pSTCurrentIPAddress;
	wxStaticText*						m_pSTCurrentSubnetMask;
	wxStaticText*						m_pSTCurrentDefaultGateway;
	wxComboBox*							m_pCBConnectedToIPAddress;
	wxStaticText*						m_pSTConnectedToNetmask;
	wxStaticText*						m_pSTConnectedToMTU;
	wxStaticText*						m_pSTConnectedToLinkSpeed;
	wxTextCtrl*							m_pTCPersistentIPAddress;
	wxTextCtrl*							m_pTCPersistentSubnetMask;
	wxTextCtrl*							m_pTCPersistentDefaultGateway;
	wxSplitterWindow*					m_pHorizontalSplitter;
	wxSplitterWindow*					m_pVerticalSplitter;
	wxImageList							m_iconList;
	static const wxTextAttr				m_ERROR_STYLE;
	MVTLI_HANDLE						m_hTLI;
	IPv4StringValidator					m_IPv4StringValidator;
	InterfaceContainer					m_TLIInterfaces;
	DeviceMap							m_devices;
	InterfaceInfo						m_interfaceInfo[DetecedDeviceInfo::MAX_INTERFACE_COUNT];
	wxDynamicLibrary					m_TLILib;
	PTLOpen								m_pTLOpen;
	PTLClose							m_pTLClose;
	PTLUpdateInterfaceList				m_pTLUpdateInterfaceList;
	PTLGetNumInterfaces					m_pTLGetNumInterfaces;
	PTLGetInterfaceID					m_pTLGetInterfaceID;
	PTLOpenInterface					m_pTLOpenInterface;
	PIFClose							m_pIFClose;
	PIFGetNumDevices					m_pIFGetNumDevices;
	PIFGetDeviceID						m_pIFGetDeviceID;
	PIFGetInfo							m_pIFGetInfo;
	PTLIMV_IFSetInterfaceParam			m_pTLIMV_IFSetInterfaceParam;
	PIFUpdateDeviceList					m_pIFUpdateDeviceList;
	PIFGetDeviceInfo					m_pIFGetDeviceInfo;
	PIFOpenDevice						m_pIFOpenDevice;
	PTLIMV_IFGetDeviceInterfaceInfo		m_pTLIMV_IFGetDeviceInterfaceInfo;
	PTLIMV_DevSetInterfaceParam			m_pTLIMV_DevSetInterfaceParam;
	PTLIMV_DevSetParam					m_pTLIMV_DevSetParam;
	PDevClose							m_pDevClose;
	PTLIMV_MACFromSerial				m_pTLIMV_MACFromSerial;
	PTLIMV_IsValidIPv4Address			m_pTLIMV_IsValidIPv4Address;
	PTLIMV_ForceIP						m_pTLIMV_ForceIP;
	PTLIMV_DoAdressesMatch				m_pTLIMV_DoAdressesMatch;

	// any class wishing to process wxWidgets events must use this macro
	DECLARE_EVENT_TABLE()
	//-----------------------------------------------------------------------------
	// IDs for the controls and the menu commands
	enum TMenuItem
	//-----------------------------------------------------------------------------
	{
		miQuit = 1,
		miAbout,
		miAction_AssignTemporaryIP,
		miAction_UpdateDeviceList,
		miSettings_UseAdvancedDeviceDiscovery
	};
	//-----------------------------------------------------------------------------
	enum TWidgetIDs
	//-----------------------------------------------------------------------------
	{
		widVerSplitter = 1,
		widHorSplitter,
		widDeviceManufacturer,
		widDeviceSerial,
		widDeviceInterfaceCount,
		widInterfaceSelector,
		widConnectedToIPAddress,
		widPersistentIPAddress,
		widPersistentSubnetMask,
		widPersistentDefaultGateway,
		widCBUsePersistentIP,
		widCBUseDHCP,
		widBtnConfigure,
		widBtnApplyChanges
	};
};

#endif // IPConfigureFrameH
