//-----------------------------------------------------------------------------
#ifndef ImageCanvasH
#define ImageCanvasH ImageCanvasH
//-----------------------------------------------------------------------------
#include "DrawingCanvas.h"
#include <map>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <vector>
#include <wx/dcclient.h>

typedef std::map<const DrawingCanvas*, AOI*> AOIContainer;

struct ImageCanvasImpl;
class PlotCanvasImageAnalysis;

//-----------------------------------------------------------------------------
class ImageCanvas : public DrawingCanvas
//-----------------------------------------------------------------------------
{
public:
	explicit ImageCanvas() {}
	explicit ImageCanvas( wxWindow* pApp, wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize,
				 long style = wxSUNKEN_BORDER, const wxString& name = wxT("ImageCanvas"), bool boActive = true );
	virtual ~ImageCanvas();
	wxString								GetCurrentPixelDataAsString( void ) const;
	wxRect									GetVisiblePartOfImage( void );
	bool									IsScaled( void ) const { return m_boScaleToClientSize; }
	bool									IsFullScreen( void ) const;
	void									SetFullScreenMode( bool boActive );
	bool									InfoOverlayActive( void ) const { return m_boShowInfoOverlay; }
	void									RefreshScrollbars( bool boMoveToMousePos = false );
	AOI*									RegisterAOI( const wxRect&, const wxColour&, const DrawingCanvas* const pOwner );
	bool									RegisterMonitorDisplay( ImageCanvas* pMonitorDisplay );
	ImageCanvas*							GetMonitorDisplay( void ) const { return m_pMonitorDisplay; }
	bool									RemoveAOI( const DrawingCanvas* const pOwner );
	//-----------------------------------------------------------------------------
	enum TSaveResult
	//-----------------------------------------------------------------------------
	{
		srOK,
		srNoImage,
		srFailedToSave
	};
	void									IncreaseSkippedCounter( size_t count );
	void									HandleMouseAndKeyboardEvents( bool boHandle );
	TSaveResult								SaveCurrentImage( const wxString& filenameAndPath, const wxString& extension ) const;
	void									SetActiveAnalysisPlot( const PlotCanvasImageAnalysis* pPlot );
	const PlotCanvasImageAnalysis*			GetActiveAnalysisPlot( void ) const { return m_pActiveAnalysisPlot; }
	bool									SetAOI( const PlotCanvasImageAnalysis* pPlot, int x, int y, int w, int h );
	bool									SetImage( const mvIMPACT::acquire::ImageBuffer* pIB, bool boMustRefresh = true );
	const mvIMPACT::acquire::ImageBuffer*	GetImage( void ) const { return m_pIB; }
	void									SetScaling( bool boOn );
	void									SetInfoOverlay( const std::vector<wxString>& infoStrings );
	void									SetInfoOverlayMode( bool boOn );
	void									ResetRequestInProgressFlag( void );
	void									SetPerformanceWarningOutput( bool boOn );
	bool									GetPerformanceWarningOutput( void ) const { return m_boShowPerformanceWarnings; }
	void									SetUserData( int userData ) { m_userData = userData; }
	int										GetUserData( void ) const { return m_userData; }
private:
	//-----------------------------------------------------------------------------
	enum
	//-----------------------------------------------------------------------------
	{
		PERFORMANCE_WARNINGS_Y_OFFSET = 20,
		SKIPPED_IMAGE_MESSAGE_Y_OFFSET = 40,
		INFO_Y_OFFSET = 60
	};
	//-----------------------------------------------------------------------------
	enum TZoomIncrementMode
	//-----------------------------------------------------------------------------
	{
		zimMultiply,
		zimDivide
	};
	//-----------------------------------------------------------------------------
	// IDs for the controls and the menu commands
	enum TMenuItem
	//-----------------------------------------------------------------------------
	{
		miPopupFitToScreen = 1,
		miPopupShowPerformanceWarnings,
		miPopupShowRequestInfoOverlay,
		miPopupFullScreen,
		miPopupSetShiftValue
	};
	ImageCanvasImpl*						m_pImpl;
	AOIContainer							m_aoiContainer;
	bool									m_boHandleMouseAndKeyboardEvents;
	bool 									m_boRefreshInProgress;
	bool									m_boSupportsFullScreenMode;
	const PlotCanvasImageAnalysis*			m_pActiveAnalysisPlot;
	bool									m_boScaleToClientSize;
	bool									m_boShowInfoOverlay;
	std::vector<wxString>					m_infoStringsOverlay;
	bool									m_boShowPerformanceWarnings;
	wxWindow*								m_pApp;
	size_t									m_skippedImages;
	size_t									m_skippedPaintEvents;
	const mvIMPACT::acquire::ImageBuffer*	m_pIB;
	double									m_currentZoomFactor;
	double									m_zoomFactor_Max;
	wxPoint									m_lastLeftMouseDownPos;
	wxPoint									m_lastViewStart;
	wxPoint									m_lastRightMouseDownPos;
	wxPoint									m_lastRightMouseDownPosRaw;
	AOI										m_AOIAtLeftMouseDown;
	AOI										m_AOIAtRightMouseDown;
	bool									m_boAOIDragInProgress;
	wxPoint									m_lastMousePos;
	wxPoint									m_lastStartPoint;
	double									m_lastScaleFactor;
	ImageCanvas*							m_pMonitorDisplay;
	AOI*									m_pVisiblePartOfImage;
	int										m_userData;

	template<typename _Ty> void				AppendYUV444DataPixelInfo( wxPoint pixel, wxString& pixelInfo, const ImageBuffer* pIB, const int order[3] ) const;
	template<typename _Ty> void				AppendYUVDataPixelInfo( wxPoint pixel, wxString& pixelInfo, const ImageBuffer* pIB ) const;
	template<typename _Ty> void				AppendUYVDataPixelInfo( wxPoint pixel, wxString& pixelInfo, const ImageBuffer* pIB ) const;
	void									BlitAOIs( wxPaintDC& dc, double scaleFactor, int bmpXOff, int bmpYOff, int bmpW, int bmpH );
	void									BlitInfoStrings( wxPaintDC& dc, double scaleFactor, int bmpScaledViewXOff, int bmpScaledViewVOff, int bmpXOff, int bmpYOff, int bmpW, int bmpH );
	void									BlitPerformanceMessages( wxPaintDC& dc, int bmpXOff, int bmpYOff, TImageBufferPixelFormat pixelFormat );
	void									ClipAOI( wxRect& rect, bool boForDragging ) const;
	void									DeleteAOIs( void );
	void									DragImageDisplay( void );
	wxPoint									GetScaledMousePos( int mouseXPos, int mouseYPos ) const;
	void									Init( const wxWindow* const pApp );
	void									SetMaxZoomFactor( const mvIMPACT::acquire::ImageBuffer* pIB, int oldMaxDim );
	TSaveResult								StoreImage( const wxImage& img, const wxString& filenameAndPath, const wxString& extension ) const;
	bool									SupportsFullScreenMode( void ) const { return m_boSupportsFullScreenMode; }
	void									UpdateZoomFactor( TZoomIncrementMode zim, double value );
	void									IncreaseShiftValue( void );
	void									DecreaseShiftValue( void );
	void									SetShiftValue( int value );
	int										GetShiftValue( void ) const;
	int										GetAppliedShiftValue( void ) const;
	void									OnKeyDown( wxKeyEvent& e );
	void									OnLeftDblClick( wxMouseEvent& );
	void									OnLeftDown( wxMouseEvent& );
	void									OnMotion( wxMouseEvent& );
	void									OnMouseWheel( wxMouseEvent& );
	void									OnPaint( wxPaintEvent& e );
	void									OnPopUpFitToScreen( wxCommandEvent& e );
	void									OnPopUpFullScreen( wxCommandEvent& e );
	void									OnPopUpSetShiftValue( wxCommandEvent& e );
	void									OnPopUpShowPerformanceWarnings( wxCommandEvent& e );
	void									OnPopUpShowRequestInfoOverlay( wxCommandEvent& e );
	void									OnRightDown( wxMouseEvent& );
	void									OnRightUp( wxMouseEvent& );
	DECLARE_EVENT_TABLE()
};

#endif // ImageCanvasH
