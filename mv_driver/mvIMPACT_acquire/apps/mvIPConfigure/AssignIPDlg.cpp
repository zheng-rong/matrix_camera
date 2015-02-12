//-----------------------------------------------------------------------------
#include <apps/Common/wxAbstraction.h>
#include "AssignIPDlg.h"
#include "IPConfigureFrame.h"
#include "wx/button.h"
#include "wx/combobox.h"
#include "wx/socket.h"
#include "wx/textctrl.h"
#include "wx/textdlg.h"

//=============================================================================
//================== Implementation LogOutputHandlerDlg =======================
//=============================================================================

BEGIN_EVENT_TABLE(AssignIPDlg, wxDialog)
	EVT_BUTTON(widBtnBuildMACAddress, AssignIPDlg::OnBtnBuildMACAddress)
	EVT_BUTTON(widBtnExecute, AssignIPDlg::OnBtnExecute)
	EVT_TEXT(widDesiredIPAddress, AssignIPDlg::OnDesiredIPAddressChanged)
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
AssignIPDlg::AssignIPDlg( IPConfigureFrame* pParent, MVTLI_HANDLE hTL, const wxString& deviceMACAddress /* = wxEmptyString */,
 const wxString& connectedIPAddress /* = wxEmptyString */ )
 : wxDialog(pParent, wxID_ANY, wxString(wxT("Assign A Temporary IPv4 Address To A Device")), wxDefaultPosition,
   wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER | wxMAXIMIZE_BOX | wxMINIMIZE_BOX), pTCDeviceMACAddress_(0),
   pTCDesiredIPAddress_(0), pTCDesiredSubnetMask_(0), pTCDesiredGateway_(0), pBtnBuildMACAddress_(0),
   pBtnExecute_(0), pCBAdapterList_(0), pParent_(pParent), hTL_(hTL)
//-----------------------------------------------------------------------------
{
	/*
		|-------------------------------------|
		| pTopDownSizer                       |
		| |---------------------------------| |
		| |          spacer                 | |
		| | pGroupBoxSizer                  | |
		| | |-----------------------------| | |
		| | | pEditElementsGridSizer      | | |
		| | | |-------------------------| | | |
		| | | |                         | | | |
		| | | |            | pSizer   | | | | |
		| | | |                         | | | |
		| | | |-------------------------| | | |
		| | |-----------------------------| | |
		| |          spacer                 | |
		| |                    pBtnExecute_ | |
		| |---------------------------------| |
		|-------------------------------------|
	*/

	wxBoxSizer* pTopDownSizer = new wxBoxSizer(wxVERTICAL);
	wxPanel* pPanel = new wxPanel(this);

	pTopDownSizer->AddSpacer( 10 );

	wxBoxSizer* pGroupBoxSizer = new wxStaticBoxSizer(wxVERTICAL, pPanel, wxT("Parameters:"));
	wxFlexGridSizer* pEditElementsGridSizer = new wxFlexGridSizer(2, 0);
	pEditElementsGridSizer->AddGrowableCol( 1, 3 );

	// row 1
	pEditElementsGridSizer->Add( new wxStaticText(pPanel, wxID_ANY, wxT("Device MAC Address:")), wxSizerFlags().Left().Center() );
	wxBoxSizer* pSizer = new wxBoxSizer(wxHORIZONTAL);
	pTCDeviceMACAddress_ = new wxTextCtrl(pPanel, widTCMACAddress, deviceMACAddress, wxDefaultPosition, wxDefaultSize, 0, MACStringValidator_);
	pBtnBuildMACAddress_ = new wxButton(pPanel, widBtnBuildMACAddress, wxT("&Build MAC Address"));
	pBtnBuildMACAddress_->SetToolTip( wxT("If you don't know the MAC address of your MATRIX VISION device\n press this button to construct it from the device serial.") );
	pSizer->Add( pTCDeviceMACAddress_, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	pSizer->Add( pBtnBuildMACAddress_, wxSizerFlags().Right() );
	pEditElementsGridSizer->Add( pSizer, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 2
	pEditElementsGridSizer->Add( new wxStaticText(pPanel, wxID_ANY, wxT("Desired IP Address:")), wxSizerFlags().Left().Center() );
	pTCDesiredIPAddress_ = new wxTextCtrl(pPanel, widDesiredIPAddress, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, IPv4StringValidator_);
	pEditElementsGridSizer->Add( pTCDesiredIPAddress_, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 3
	pEditElementsGridSizer->Add( new wxStaticText(pPanel, wxID_ANY, wxT("Desired Subnet Mask:")), wxSizerFlags().Left().Center() );
	pTCDesiredSubnetMask_ = new wxTextCtrl(pPanel, widDesiredSubnetMask, wxT("255.255.255.0"), wxDefaultPosition, wxDefaultSize, 0, IPv4StringValidator_);
	pEditElementsGridSizer->Add( pTCDesiredSubnetMask_, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	// row 4
	pEditElementsGridSizer->Add( new wxStaticText(pPanel, wxID_ANY, wxT("Desired Gateway:")), wxSizerFlags().Left().Center() );
	pTCDesiredGateway_ = new wxTextCtrl(pPanel, widDesiredGateway, wxT("0.0.0.0"), wxDefaultPosition, wxDefaultSize, 0, IPv4StringValidator_);
	pEditElementsGridSizer->Add( pTCDesiredGateway_, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );
	pGroupBoxSizer->Add( pEditElementsGridSizer, wxSizerFlags().Align( wxGROW ) );
	// row 5
	pEditElementsGridSizer->Add( new wxStaticText(pPanel, wxID_ANY, wxT("Local IP To Use For Sending:")), wxSizerFlags().Left().Center() );
	pCBAdapterList_ = new wxComboBox(pPanel, widAdapterList, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, 0, wxCB_DROPDOWN);
	pEditElementsGridSizer->Add( pCBAdapterList_, wxSizerFlags(2).Align( wxGROW | wxALIGN_CENTER_VERTICAL ) );

	const InterfaceContainer& interfaces(pParent->GetInterfaces());
	InterfaceContainer::const_iterator it = interfaces.begin();
	const InterfaceContainer::const_iterator itEND = interfaces.end();
	while( it != itEND )
	{
		pCBAdapterList_->AppendString( ConvertedString(pParent->GetInterfaceStringInfo( it->second, INTERFACE_INFO_IP_STRING )) );
		++it;
	}

	const unsigned int cnt = pCBAdapterList_->GetCount();
	if( cnt > 0 )
	{
		bool boSelected = false;
		if( !connectedIPAddress.IsEmpty() )
		{
			for( unsigned int i=0; i<cnt; i++ )
			{
				if( connectedIPAddress == pCBAdapterList_->GetString( i ) )
				{
					pCBAdapterList_->Select( i );
					boSelected = true;
					break;
				}
			}
		}
		if( !boSelected )
		{
			pCBAdapterList_->Select( 0 );
		}
	}

	pTopDownSizer->Add( pGroupBoxSizer, wxSizerFlags().Expand() );
	pTopDownSizer->AddSpacer( 10 );
	pBtnExecute_ = new wxButton(pPanel, widBtnExecute, wxT("&Execute"));
	pTopDownSizer->Add( pBtnExecute_, wxSizerFlags().Right().Border( wxALL, 5 ) );
	pTopDownSizer->AddSpacer( 5 );
	pPanel->SetSizer( pTopDownSizer );
	pTopDownSizer->SetSizeHints( this );
}

//-----------------------------------------------------------------------------
void AssignIPDlg::OnBtnBuildMACAddress( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	wxTextEntryDialog dlg(this,
		wxT("Please note that only serial numbers of MATRIX VISION GmbH\n")
		wxT("devices can be converted using this dialog"),
		wxT("Please enter the serial number of the device"),
		wxT(""),
		wxOK | wxCANCEL);

	if( dlg.ShowModal() == wxID_OK )
	{
		size_t bufSize = 0;
		int result = pParent_->MACFromSerial( dlg.GetValue().mb_str(), 0, &bufSize );
		if( result != 0 )
		{
			wxMessageBox(wxT("Failed to query the result size for a string buffer. Either the serial number\nEntered was invalid or another problem has been detected.\nMore information can be found in the log files"), wxT("ERROR"), wxOK | wxICON_ERROR, this);
			return;
		}

		char* pBuf = new char[bufSize];
		memset( pBuf, 0, bufSize );
		result = pParent_->MACFromSerial( dlg.GetValue().mb_str(), pBuf, &bufSize );
		pTCDeviceMACAddress_->SetValue( ConvertedString(pBuf) );
		delete [] pBuf;
		if( result != 0 )
		{
			wxMessageBox(wxT("Failed to convert serial number\nMore information can be found in the log files"), wxT("ERROR"), wxOK | wxICON_ERROR, this);
			return;
		}
	}
}

//-----------------------------------------------------------------------------
void AssignIPDlg::OnBtnExecute( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	wxString adapter(( pCBAdapterList_->GetCount() > 0 ) ? pCBAdapterList_->GetValue() : wxString(wxEmptyString));

	wxString MACAddress = pTCDeviceMACAddress_->GetValue();
	if( MACAddress.IsEmpty() )
	{
		wxMessageBox(wxT("Please specify a device MAC address."), wxT("Command execution failed"), wxOK | wxICON_EXCLAMATION, this);
		return;
	}

	MACAddress.Replace( wxT(":"), wxT("") );
	if( MACAddress.Len() > 12 )
	{
		wxMessageBox(wxString::Format( wxT("'%s' is not valid a valid MAC address. Can't execute command."), MACAddress.c_str()), wxT("Command execution failed"), wxOK | wxICON_EXCLAMATION, this);
		return;
	}

	wxString IPAddress = pTCDesiredIPAddress_->GetValue();
	wxString SubnetMask = pTCDesiredSubnetMask_->GetValue();
	wxString Gateway = pTCDesiredGateway_->GetValue();
	if( IPAddress.StartsWith( wxT("169.254." ) ) &&
		Gateway.IsEmpty() )
	{
		Gateway = wxT("0.0.0.0");
	}

	if( !ValidateIPDataSet( IPAddress, SubnetMask, Gateway, this, pParent_ ) )
	{
		return;
	}

	int result = pParent_->ForceIP( MACAddress.mb_str(),
		IPAddress.mb_str(),
		SubnetMask.mb_str(),
		Gateway.mb_str(),
		adapter.mb_str(),
		1000 );

	if( result != 0 )
	{
		wxMessageBox(wxString::Format( wxT("The execution of the command failed(error code: %d), More information can be found in the log file."), result ), wxT("Command execution failed"), wxOK | wxICON_ERROR, this);
	}

	EndModal( ( result == 0 ) ? wxID_OK : wxID_CANCEL );
}

//-----------------------------------------------------------------------------
void AssignIPDlg::OnDesiredIPAddressChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	if( pTCDesiredIPAddress_ && pTCDesiredSubnetMask_ && pTCDesiredGateway_ )
	{
		wxString IPv4Address(pTCDesiredIPAddress_->GetValue());
		if( IPv4Address == wxT("169.254.") )
		{
			pTCDesiredSubnetMask_->SetValue( wxT("255.255.0.0") );
			pTCDesiredGateway_->SetValue( wxT("") );
		}
		else if( IPv4Address == wxT("192.168.") )
		{
			pTCDesiredSubnetMask_->SetValue( wxT("255.255.255.0") );
			pTCDesiredGateway_->SetValue( wxT("192.168.0.15") );
		}
		else if( !IPv4Address.StartsWith( wxT("169.254.") ) &&
			!IPv4Address.StartsWith( wxT("192.168.") ) )
		{
			pTCDesiredGateway_->SetValue( wxT("0.0.0.0") );
		}
	}
}

//-----------------------------------------------------------------------------
bool ValidateIPDataSet( const wxString& IPAddress, const wxString& SubnetMask, const wxString& Gateway, wxWindow* pParent, IPConfigureFrame* pMainFrame )
//-----------------------------------------------------------------------------
{
	wxString errorCaption(wxT("Parameter validation check failed"));
	if( IPAddress.IsEmpty() )
	{
		wxMessageBox(wxT("Please specify a valid IP address."), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
		return false;
	}

	unsigned long firstIPByte = 0;
	if( ( pMainFrame->IsValidIPv4Address( IPAddress.mb_str() ) != 0 ) ||
		!IPAddress.BeforeFirst( wxT('.') ).ToULong( &firstIPByte ) )
	{
		wxMessageBox(wxString::Format( wxT("'%s' is not a valid IP address. Can't execute command."), IPAddress.c_str()), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
		return false;
	}

	if( ( firstIPByte < 1 ) || ( firstIPByte > 223 ) )
	{
		wxMessageBox(wxString::Format( wxT("'%s' is not a valid IP address. The first byte of the IP address must be a value between 1 and 223. Can't execute command."), IPAddress.c_str()), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
		return false;
	}

	if( SubnetMask.IsEmpty() )
	{
		wxMessageBox(wxT("Please specify a valid subnet mask."), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
		return false;
	}

	if( pMainFrame->IsValidIPv4Address( SubnetMask.mb_str() ) != 0 )
	{
		wxMessageBox(wxString::Format( wxT("'%s' is not a valid netmask. Can't execute command."), SubnetMask.c_str()), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
		return false;
	}

	if( !IPAddress.StartsWith( wxT("169.254.") ) )
	{
		if( Gateway.IsEmpty() )
		{
			wxMessageBox(wxT("Please specify a valid gateway address."), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
			return false;
		}

		if( ( pMainFrame->IsValidIPv4Address( Gateway.mb_str() ) != 0 ) ||
			!Gateway.BeforeFirst( wxT('.') ).ToULong( &firstIPByte ) )
		{
			wxMessageBox(wxString::Format( wxT("'%s' is not a valid gateway. Can't execute command."), Gateway.c_str()), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
			return false;
		}

		if( ( firstIPByte < 1 ) || ( firstIPByte > 223 ) )
		{
			wxMessageBox(wxString::Format( wxT("'%s' is not a valid gateway. The first byte of the gateway must be a value between 1 and 223. Can't execute command."), Gateway.c_str()), errorCaption, wxOK | wxICON_EXCLAMATION, pParent);
			return false;
		}
	}

	return true;
}
