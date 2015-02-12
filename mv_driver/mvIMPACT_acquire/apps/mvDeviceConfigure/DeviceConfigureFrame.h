//-----------------------------------------------------------------------------
#ifndef DeviceConfigureFrameH
#define DeviceConfigureFrameH DeviceConfigureFrameH
//-----------------------------------------------------------------------------
#include "wx/wx.h"
#include "DebugFileParser.h"
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "ObjectFactory.h"
#include "DeviceHandler.h"
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#	include "win32/DirectShowSupport.h"
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
#	include <common/ProcessorPowerStateConfiguration/ProcessorPowerStateConfiguration.h>
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
#include <map>

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

class DeviceListCtrl;

//-----------------------------------------------------------------------------
class DeviceConfigureFrame : public wxFrame
//-----------------------------------------------------------------------------
{
public:
	explicit DeviceConfigureFrame( const wxString& title, const wxPoint& pos, const wxSize& size, int argc, wxChar** argv );
	   ~DeviceConfigureFrame();
	typedef DeviceHandler* (*CreateDeviceHandler)(mvIMPACT::acquire::Device*);
	typedef ObjectFactory<DeviceHandler, wxString, CreateDeviceHandler, mvIMPACT::acquire::Device*> DeviceHandlerFactory;
	void ActivateDeviceInPropView( int deviceIndex );
	int SetID( int deviceIndex );
	int UpdateFirmware( int deviceIndex );
	int UpdateKernelDriver( int deviceIndex );
	void UpdateMenu( int deviceIndex );
	void WriteErrorMessage( const wxString& msg );
	void WriteLogMessage( const wxString& msg, const wxTextAttr& style = wxTextAttr(wxColour(0, 0, 0)) );
	DeviceManager& GetDeviceManager( void ) { return m_devMgr; }
	DeviceHandlerFactory& GetHandlerFactory( void ) { return m_deviceHandlerFactory; }
	int UpdateDMABufferSize( int deviceIndex );
protected:
	// event handlers (these functions should _not_ be virtual)
	void OnAbout( wxCommandEvent& e );
	void OnConfigureLogOutput( wxCommandEvent& e );
	void OnQuit( wxCommandEvent& e );
	void OnSetID( wxCommandEvent& e );
	void OnTimer( wxTimerEvent& e );
	void OnUpdateDeviceList( wxCommandEvent& e );
	void OnUpdateFirmware( wxCommandEvent& e );
	void OnUpdateKernelDriver( wxCommandEvent& e );
	void OnUpdateDMABufferSize( wxCommandEvent& e );
	//-----------------------------------------------------------------------------
	enum TTimerEvent
	//-----------------------------------------------------------------------------
	{
		teListUpdate,
		teTimer
	};
	//-----------------------------------------------------------------------------
	enum
	//-----------------------------------------------------------------------------
	{
		TIMER_PERIOD = 500
	};
private:
	void								BuildList( void );
	int									SetID( Device* pDev, int newID );
	void								UpdateDeviceList( void );
	int									UpdateFirmware( Device* pDev, bool boSilentMode );
	int									UpdateKernelDriver( Device* pDev, bool boSilentMode );
	mvIMPACT::acquire::DeviceManager	m_devMgr;
	DeviceListCtrl*						m_pDevListCtrl;
	wxMenuItem*							m_pMIActionSetID;
	wxMenuItem*							m_pMIActionUpdateFW;
	wxMenuItem*							m_pMIActionUpdateKernelDriver;
	wxMenuItem*							m_pMIActionUpdateDMABufferSize;
	wxMenuItem*							m_pMIActionUpdateDeviceList;
	wxTextCtrl*							m_pLogWindow;
	wxTimer								m_timer;
	wxTimer								m_listUpdateTimer;
	unsigned int						m_lastDevMgrChangedCount;
	LogConfigurationVector				m_debugData;
	DeviceHandlerFactory				m_deviceHandlerFactory;
	wxString							m_customFirmwarePath;
	std::string							m_IPv4Mask;
	//-----------------------------------------------------------------------------
	struct DeviceConfigurationData
	//-----------------------------------------------------------------------------
	{
		std::string searchToken_;
		bool boUpdateKernelDriver_;
		bool boUpdateFW_;
		bool boSetDeviceID_;
		int deviceID_;
		explicit DeviceConfigurationData( const std::string& searchToken ) : searchToken_(searchToken),
			boUpdateKernelDriver_(false), boUpdateFW_(false),
			boSetDeviceID_(false), deviceID_(-1) {}
	};
	std::map<wxString, DeviceConfigurationData>::iterator GetConfigurationEntry( wxString& value );
	std::map<wxString, DeviceConfigurationData>	m_devicesToConfigure;
	bool m_boPendingQuit;
	// any class wishing to process wxWidgets events must use this macro
	DECLARE_EVENT_TABLE()
	//-----------------------------------------------------------------------------
	// IDs for the controls and the menu commands
	enum TMenuItem
	//-----------------------------------------------------------------------------
	{
		miQuit = 1,
		miAbout,
		miAction_SetID = 1000,
		miAction_UpdateFW,
		miAction_UpdateKernelDriver,
		miAction_ConfigureLogOutput,
		miAction_UpdateDeviceList,
		miAction_UpdateDMABufferSize,
		miCOMMON_LAST
	};
	//-----------------------------------------------------------------------------
	enum TDeviceIDLimits
	//-----------------------------------------------------------------------------
	{
		didlMin = 0,
		didlMax = 250
	};
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
	DirectShowDeviceManager				m_DSDevMgr;
	//-----------------------------------------------------------------------------
	enum TMenuItem_DirectShow
	//-----------------------------------------------------------------------------
	{
		miDirectShow_RegisterAllDevices = miCOMMON_LAST,
		miDirectShow_UnregisterAllDevices,
		miDirectShow_SetFriendlyName
	};
	wxMenuItem*							m_pMIDirectShow_SetFriendlyName;
	void OnRegisterAllDevicesForDirectShow( wxCommandEvent& )
	{
		m_DSDevMgr.registerAllDevices();
		BuildList();
	}
	void OnUnregisterAllDevicesForDirectShow( wxCommandEvent& )
	{
		m_DSDevMgr.unregisterAllDevices();
		BuildList();
	}
	void OnSetFriendlyNameForDirectShow( wxCommandEvent& e );
public:
	int SetDSFriendlyName( int deviceIndex );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
	//-----------------------------------------------------------------------------
	enum TMenuItem_Settings
	//-----------------------------------------------------------------------------
	{
		miSettings_CPUIdleStatesEnabled = miCOMMON_LAST + 100
	};
	wxMenuItem*							m_pMISettings_CPUIdleStatesEnabled;
	void OnSettings_CPUIdleStatesEnabled( wxCommandEvent& e )
	{
		if( wxMessageBox( wxString::Format( wxT("Are you sure you want to %s the C1, C2 and C3 states for ALL processors in this system?"), e.IsChecked() ? wxT("enable") : wxT("disable") ), wxT("CPU Power State Configuration"), wxYES_NO | wxNO_DEFAULT | wxICON_EXCLAMATION, this ) == wxYES )
		{
			if( !SetPowerState( e.IsChecked() ) )
			{
				wxMessageBox( wxString::Format( wxT("Failed to %s the C1, C2 and C3 states for ALL processors."), e.IsChecked() ? wxT("enable") : wxT("disable") ), wxT("ERROR"), wxOK | wxICON_ERROR, this );
			}
		}
		else
		{
			bool boValue = false;
			if( GetPowerState( boValue ) )
			{
				m_pMISettings_CPUIdleStatesEnabled->Check( boValue );
			}
			else
			{
				wxMessageBox( wxT("Failed to query the C1, C2 and C3 states for ALL processors."), wxT("ERROR"), wxOK | wxICON_ERROR, this );
			}
		}
	}
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
};

#endif // DeviceConfigureFrameH
