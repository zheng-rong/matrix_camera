#include "PlotCanvas.h"

//=============================================================================
//================= Implementation PlotCanvas =================================
//=============================================================================
BEGIN_EVENT_TABLE(PlotCanvas, DrawingCanvas)
  EVT_PAINT(PlotCanvas::OnPaint)
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
PlotCanvas::PlotCanvas( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
					   const wxSize& size /* = wxDefaultSize */, long style /* = wxSUNKEN_BORDER */,
					   const wxString& name /* = "DrawingCanvas" */, bool boActive /* = false */ ) : 
	DrawingCanvas(parent, id, pos, size, style, name, boActive), m_BorderWidth(25), m_ImageCount(-1), m_UpdateFrequency(3)
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
void PlotCanvas::DrawInfoString( wxPaintDC& dc, const wxString& info, wxCoord& xOffset, wxCoord yOffset, const wxColour& colour ) const
//-----------------------------------------------------------------------------
{
	static const wxCoord rectSize(10);

	dc.DrawText( info, xOffset, yOffset );
	wxCoord textWidth, textHeight;
	dc.GetTextExtent( info, &textWidth, &textHeight );
	wxCoord yOffsetRect(((textHeight-rectSize)/2)+1);
	xOffset += textWidth;
	dc.SetPen( wxPen(*wxBLACK, 1, wxSOLID ) );
	dc.SetBrush( wxBrush(colour) );
	dc.DrawRectangle( xOffset, yOffsetRect, rectSize, rectSize );
	dc.SetBrush( wxNullBrush );
	dc.SetPen( wxNullPen );
	xOffset += rectSize + 2;
}

//-----------------------------------------------------------------------------
void PlotCanvas::DrawMarkerLines( wxPaintDC& dc, const wxCoord w, const wxCoord h, const double scaleX ) const
//-----------------------------------------------------------------------------
{
	const int borderWidth = GetBorderWidth();

	// white rectangle for the background
	dc.SetPen( *wxBLACK );
	dc.SetBrush( *wxWHITE_BRUSH );
	dc.DrawRectangle( 0, 0, w, h );
	dc.SetBrush( wxNullBrush );
	// lower line
	dc.DrawLine( borderWidth, h-borderWidth, w-borderWidth, h-borderWidth );
	// line on left side
	dc.DrawLine( borderWidth, h-borderWidth, borderWidth, borderWidth );

	// markers for X-axis
	int markerStartHeight = h - borderWidth - 3;
	int markerEndHeight = h - borderWidth + 4;
	unsigned int from = 0, to = 0;
	const unsigned int XMarkerStepWidth = GetXMarkerParameters( from, to );
	wxCoord textWidth;
	for( unsigned int i=from; i<to; i+=XMarkerStepWidth )
	{
		const int xVal = i - from;
		dc.DrawLine( static_cast<int>( borderWidth + ( xVal * scaleX )), markerStartHeight, static_cast<int>( borderWidth + ( xVal * scaleX )), markerEndHeight );
		const wxString XMarkerString(wxString::Format( wxT("%d"), i ));
		dc.GetTextExtent( XMarkerString, &textWidth, 0 );
		dc.DrawText( XMarkerString, static_cast<int>(borderWidth + ( xVal * scaleX ) - ( textWidth / 2 )), h - borderWidth + 3 );
	}

	// markers for y-axis
	int markerStart = borderWidth - 3;
	int markerEnd = borderWidth + 4;
	dc.DrawLine( markerStart, borderWidth, markerEnd, borderWidth );
}

//-----------------------------------------------------------------------------
void PlotCanvas::DrawProfileLine( wxPaintDC& dc, int h, int startOffset, double scaleX, double scaleY, unsigned int from, unsigned int to, int* pData, int elementCount, const wxColour& colour ) const
//-----------------------------------------------------------------------------
{
	dc.SetPen( colour );
	int lowerStart = h - GetBorderWidth();
	const int maxY = static_cast<int>(to - from);
	for( int i=0; i<elementCount-1; i++ )
	{
		dc.DrawLine( static_cast<int>( startOffset + ( CalculateXStart( from, to, i, pData[i], i+1, pData[i+1], i, pData[i] ) * scaleX ) ),
					 static_cast<int>( lowerStart - ( saveAssign( pData[i] - static_cast<int>(from), 0, maxY ) * scaleY ) ),
					 static_cast<int>( startOffset + ( CalculateXStart( from, to, i, pData[i], i+1, pData[i+1], i+1, pData[i+1] ) * scaleX ) ),
					 static_cast<int>( lowerStart - ( saveAssign( pData[i+1] - static_cast<int>(from), 0, maxY ) * scaleY ) ) );
	}
	dc.SetPen( wxNullPen );
}

//-----------------------------------------------------------------------------
void PlotCanvas::OnPaint( wxPaintEvent& )
//-----------------------------------------------------------------------------
{
	wxCriticalSectionLocker locker(m_critSect);
	wxPaintDC dc(this);

	if( !IsActive() )
	{
		return;
	}

	OnPaintCustom( dc );

	dc.SetBrush( wxNullBrush );
	dc.SetPen( wxNullPen );
}

//-----------------------------------------------------------------------------
void PlotCanvas::SetBorderWidth( int borderWidth )
//-----------------------------------------------------------------------------
{
	wxCriticalSectionLocker locker(m_critSect);
	m_BorderWidth = borderWidth;
}

//-----------------------------------------------------------------------------
bool PlotCanvas::SetUpdateFrequency( int frequency )
//-----------------------------------------------------------------------------
{
	if( ( frequency >= ulMin ) && ( frequency <= ulMax ) )
	{
		m_UpdateFrequency = frequency;
		return true;
	}
	return false;
}
