//-----------------------------------------------------------------------------
#include "DeviceListCtrl.h"
#include "IPConfigureFrame.h"

extern IPConfigureFrame* g_pFrame;

//-----------------------------------------------------------------------------
const DetecedDeviceInfo* GetDeviceInfo( long item, const DeviceMap& m )
//-----------------------------------------------------------------------------
{
	DeviceMap::const_iterator it = m.begin();
	DeviceMap::const_iterator itEND = m.end();
	while( it != itEND )
	{
		if( it->second->id_ == item )
		{
			return it->second;
		}
		++it;
	}
	return 0;
}

//-----------------------------------------------------------------------------
int wxCALLBACK ListCompareFunction( long item1, long item2, long column )
//-----------------------------------------------------------------------------
{
	if( g_pFrame )
	{
		const DeviceMap& m(g_pFrame->GetDeviceMap());
		const DetecedDeviceInfo* p1 = GetDeviceInfo( item1, m );
		const DetecedDeviceInfo* p2 = GetDeviceInfo( item2, m );
		if( p1 && p2 )
		{
			switch( column )
			{
			case lcProduct:
				return p1->modelName_.compare( p2->modelName_ );
			case lcSerial:
				return p1->deviceSerial_.compare( p2->deviceSerial_ );
			case lcPrimaryInterfaceIPAddress:
				return p1->interfaceInfo_[0].currentIPAddress_.compare( p2->interfaceInfo_[0].currentIPAddress_ );
			default:
				break;
			}
		}
	}
	return 0;
}

BEGIN_EVENT_TABLE(DeviceListCtrl, wxListCtrl)
	EVT_LIST_COL_CLICK(LIST_CTRL, DeviceListCtrl::OnColClick)
	EVT_LIST_DELETE_ALL_ITEMS(LIST_CTRL, DeviceListCtrl::OnDeleteAllItems)
	EVT_LIST_ITEM_SELECTED(LIST_CTRL, DeviceListCtrl::OnSelected)
	EVT_LIST_ITEM_DESELECTED(LIST_CTRL, DeviceListCtrl::OnDeselected)
	EVT_LIST_ITEM_RIGHT_CLICK(LIST_CTRL, DeviceListCtrl::OnItemRightClick)
	EVT_MENU(LIST_ASSIGN_TEMPORARY_IP, DeviceListCtrl::OnAssignTemporaryIP)
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
DeviceListCtrl::DeviceListCtrl(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style, IPConfigureFrame* pParentFrame )
    : wxListCtrl(parent, id, pos, size, style), m_pParentFrame(pParentFrame), m_selectedItemID(-1) {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnColClick( wxListEvent& e )
//-----------------------------------------------------------------------------
{
	SortItems( ListCompareFunction, e.GetColumn() );
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnDeleteAllItems( wxListEvent& )
//-----------------------------------------------------------------------------
{
	m_selectedItemID = -1;
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnDeselected( wxListEvent& e )
//-----------------------------------------------------------------------------
{
	m_selectedItemID = -1;
	if( m_pParentFrame )
	{
		m_pParentFrame->OnListItemDeselected( e.GetIndex() );
	}
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnItemRightClick( wxListEvent& e )
//-----------------------------------------------------------------------------
{
	wxMenu menu(wxT(""));
	menu.Append( LIST_ASSIGN_TEMPORARY_IP, wxT("&Assign Temporary IPv4 Address"), wxEmptyString );
	PopupMenu( &menu, e.GetPoint() );
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnAssignTemporaryIP( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
	if( m_pParentFrame )
	{
		m_pParentFrame->AssignTemporaryIP( m_selectedItemID );
	}
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::SetCurrentItemIndex( int index )
//-----------------------------------------------------------------------------
{
	m_selectedItemID = index;
	if( m_pParentFrame )
	{
		m_pParentFrame->OnListItemSelected( m_selectedItemID );
	}
}
