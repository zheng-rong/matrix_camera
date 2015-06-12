#ifndef linux
#   error Sample only for linux side!!
#endif  // linux
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

#ifdef MALLOC_TRACE
#   include <mcheck.h>
#endif  // MALLOC_TRACE

#define PRESS_A_KEY_AND_RETURN          \
    cout << "Press a key..." << endl;   \
    getchar(); \
    return 0;

using namespace std;
using namespace mvIMPACT::acquire;

//-----------------------------------------------------------------------------
static unsigned int liveLoop( Device* pDev, bool boStoreFrames, const string& settingName, bool boSingleShotMode )
//-----------------------------------------------------------------------------
{
    cout << " == " << __FUNCTION__ << " - establish access to the statistic properties...." << endl;
    // establish access to the statistic properties
    Statistics statistics( pDev );
    cout << " == " << __FUNCTION__ << " - create an interface to the device found...." << endl;
    // create an interface to the device found
    FunctionInterface fi( pDev );

    if( !settingName.empty() )
    {
        cout << "Trying to load setting " << settingName << "..." << endl;
        int result = fi.loadSetting( settingName );
        if( result != DMR_NO_ERROR )
        {
            cout << "loadSetting( \"" << settingName << "\" ); call failed: " << ImpactAcquireException::getErrorCodeAsString( result ) << endl;
        }
    }

#if 0
    // if running mvBlueFOX on an embedded system (e.g. ARM) with USB 1.1 it may be necessary to change
    // a few settings and timeouts like this:

    // get other settings
    SettingsBlueFOX setting( pDev );

    // set request timeout higher because USB 1.1 on ARM is soooo slow
    setting.cameraSetting.imageRequestTimeout_ms.write( 5000 );
    // use on Demand mode
    setting.cameraSetting.triggerMode.write( ctmOnDemand );
#endif

#if 0
    // this section contains special settings that might be interesting for mvBlueCOUGAR or mvBlueLYNX-M7
    // related embedded devices
    CameraSettingsBlueCOUGAR cs( pDev );
    int maxWidth = cs.aoiWidth.getMaxValue();
    cs.aoiWidth.write( maxWidth );
    //cs.autoGainControl.write( agcOff );
    //cs.autoExposeControl.write( aecOff );
    //cs.exposeMode.write( cemOverlapped );
    //cs.pixelClock_KHz.write( cpc40000KHz );
    //cs.expose_us.write( 5000 );
#endif

    // If this is color sensor, we will NOT convert the Bayer data into a RGB image as this
    // will cost a lot of time on an embedded system
    ImageProcessing ip( pDev );
    if( ip.colorProcessing.isValid() )
    {
        ip.colorProcessing.write( cpmRaw );
    }

    SystemSettings ss( pDev );
    // Prefill the capture queue with ALL buffers currently available. In case the acquisition engine is operated
    // manually, buffers can only be queued when they have been queued before the acquisition engine is started as well.
    // Even though there can be more then 1, for this sample we will work with the default capture queue
    int requestResult = DMR_NO_ERROR;
    int requestCount = 0;

    if( boSingleShotMode )
    {
        fi.imageRequestSingle();
        ++requestCount;
    }
    else
    {
        while( ( requestResult = fi.imageRequestSingle() ) == DMR_NO_ERROR )
        {
            ++requestCount;
        }
    }

    if( requestResult != DEV_NO_FREE_REQUEST_AVAILABLE )
    {
        cout << "Last result: " << requestResult << "(" << ImpactAcquireException::getErrorCodeAsString( requestResult ) << "), ";
    }
    cout << requestCount << " buffers requested";

    if( ss.requestCount.hasMaxValue() )
    {
        cout << ", max request count: " << ss.requestCount.getMaxValue();
    }
    cout << endl;

    const bool boManualAcquisitionEngineControl = pDev->acquisitionStartStopBehaviour.isValid() && ( pDev->acquisitionStartStopBehaviour.read() == assbUser );
    if( boManualAcquisitionEngineControl )
    {
        cout << "Manual start/stop of acquisition engine requested." << endl;
        const int startResult = fi.acquisitionStart();
        cout << "Result of start: " << startResult << "("
             << ImpactAcquireException::getErrorCodeAsString( startResult ) << ")" << endl;
    }
    cout << "Press <<ENTER>> to end the application!!" << endl;

    // run thread loop
    const Request* pRequest = 0;
    const unsigned int timeout_ms = 8000;   // USB 1.1 on an embedded system needs a large timeout for the first image
    int requestNr = -1;
    bool boLoopRunning = true;
    unsigned int cnt = 0;
    while( boLoopRunning )
    {
        // wait for results from the default capture queue
        requestNr = fi.imageRequestWaitFor( timeout_ms );
        if( fi.isRequestNrValid( requestNr ) )
        {
            pRequest = fi.getRequest( requestNr );
            if( pRequest->isOK() )
            {
                ++cnt;
                // here we can display some statistical information every 100th image
                if( cnt % 100 == 0 )
                {
                    cout << cnt << ": Info from " << pDev->serial.read()
                         << ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
                         << ", " << statistics.errorCount.name() << ": " << statistics.errorCount.readS()
                         << ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS() << " Image count: " << cnt
                         << " (dimensions: " << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << ", format: " << pRequest->imagePixelFormat.readS();
                    if( pRequest->imageBayerMosaicParity.read() != bmpUndefined )
                    {
                        cout << ", " << pRequest->imageBayerMosaicParity.name() << ": " << pRequest->imageBayerMosaicParity.readS();
                    }
                    cout << "), line pitch: " << pRequest->imageLinePitch.read() << endl;
                    if( boStoreFrames )
                    {
                        ostringstream oss;
                        oss << "Image" << cnt << "." << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << "." << pRequest->imagePixelFormat.readS();
                        if( pRequest->imageBayerMosaicParity.read() != bmpUndefined )
                        {
                            oss << "(BayerPattern=" << pRequest->imageBayerMosaicParity.readS() << ")";
                        }
                        oss << ".raw";
                        FILE* fp = fopen( oss.str().c_str(), "wb" );
                        if( fp )
                        {
                            unsigned char* pImageData = ( unsigned char* ) pRequest->imageData.read();
                            for( int h = 0; h < pRequest->imageHeight.read(); h++ )
                            {
                                // write one line
                                fwrite( pImageData, pRequest->imageWidth.read(), pRequest->imageBytesPerPixel.read(), fp );
                                // respect image line pitch
                                pImageData += pRequest->imageLinePitch.read();
                            }
                            fclose( fp );
                        }
                    }
                }
            }
            else
            {
                cout << "*** Error: A request has been returned with the following result: " << pRequest->requestResult << endl;
            }

            // this image has been displayed thus the buffer is no longer needed...
            fi.imageRequestUnlock( requestNr );
            // send a new image request into the capture queue
            fi.imageRequestSingle();
            if( boManualAcquisitionEngineControl && boSingleShotMode )
            {
                const int startResult = fi.acquisitionStart();
                if( startResult != DMR_NO_ERROR )
                {
                    cout << "Result of start: " << startResult << "("
                         << ImpactAcquireException::getErrorCodeAsString( startResult ) << ")" << endl;
                }
            }
        }
        else
        {
            cout << "*** Error: Result of waiting for a finished request: " << requestNr << "("
                 << ImpactAcquireException::getErrorCodeAsString( requestNr ) << "). Timeout value too small?" << endl;
        }

        boLoopRunning = waitForInput( 0, STDOUT_FILENO ) == 0 ? true : false; // break by STDIN
    }

    if( boManualAcquisitionEngineControl && !boSingleShotMode )
    {
        const int stopResult = fi.acquisitionStop();
        cout << "Manually stopping acquisition engine. Result: " << stopResult << "("
             << ImpactAcquireException::getErrorCodeAsString( stopResult ) << ")" << endl;
    }
    cout << " == " << __FUNCTION__ << " - free resources...." << endl;
    // free resources
    fi.imageRequestReset( 0, 0 );
    return 0;
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
#ifdef MALLOC_TRACE
    mtrace();
#endif  // MALLOC_TRACE
    cout << " ++ starting application...." << endl;

    bool boStoreFrames = false;
    string settingName;
    int width = -1;
    int height = -1;
    string pixelFormat;
    string acquisitionMode;
    string deviceSerial;
    int defaultRequestCount = -1;
    for( int i = 1; i < argc; i++ )
    {
        string arg( argv[i] );
        if( arg == "-sf" )
        {
            boStoreFrames = true;
        }
        else if( arg.find( "-a" ) == 0 )
        {
            acquisitionMode = arg.substr( 2 );
        }
        else if( arg.find( "-drc" ) == 0 )
        {
            defaultRequestCount = atoi( arg.substr( 4 ).c_str() );
        }
        else if( arg.find( "-h" ) == 0 )
        {
            height = atoi( arg.substr( 2 ).c_str() );
        }
        else if( arg.find( "-p" ) == 0 )
        {
            pixelFormat = arg.substr( 2 );
        }
        else if( arg.find( "-s" ) == 0 )
        {
            deviceSerial = arg.substr( 2 );
        }
        else if( arg.find( "-w" ) == 0 )
        {
            width = atoi( arg.substr( 2 ).c_str() );
        }
        else
        {
            // try to load this setting later on...
            settingName = string( argv[1] );
        }
    }

    if( argc <= 1 )
    {
        cout << "Available command line parameters:" << endl
             << endl
             << "-sf to store every 100th frame in raw format" << endl
             << "-a<mode> to set the acquisition mode" << endl
             << "-h<height> to set the AOI width" << endl
             << "-p<pixelFormat> to set the pixel format" << endl
             << "-s<serialNumber> to pre-select a certain device. If this device can be found no further user interaction is needed" << endl
             << "-w<width> to set the AOI width" << endl
             << "-drc<bufferCount> to specify the default request count" << endl
             << "any other string will be interpreted as a name of a setting to load" << endl;
    }

    DeviceManager devMgr;
    Device* pDev = 0;
    if( !deviceSerial.empty() )
    {
        pDev = devMgr.getDeviceBySerial( deviceSerial );
        if( pDev )
        {
            switchFromGenericToGenICamInterface( pDev );
        }
    }
    if( !pDev )
    {
        pDev = getDeviceFromUserInput( devMgr );
    }

    // create an interface to the first MATRIX VISION device with the serial number sDevSerial
    if( pDev )
    {
        cout << "Initialising device: " << pDev->serial.read() << ". This might take some time..." << endl
             << "Using interface layout '" << pDev->interfaceLayout.readS() << "'." << endl;
        try
        {
            if( defaultRequestCount > 0 )
            {
                cout << "Setting default request count to " << defaultRequestCount << endl;
                pDev->defaultRequestCount.write( defaultRequestCount );
            }
            pDev->open();
            switch( pDev->interfaceLayout.read() )
            {
            case dilGenICam:
                {
                    mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );
                    mvIMPACT::acquire::GenICam::AcquisitionControl ac( pDev );
                    if( width > 0 )
                    {
                        ifc.width.write( width );
                    }
                    if( height > 0 )
                    {
                        ifc.height.write( height );
                    }
                    if( !pixelFormat.empty() )
                    {
                        ifc.pixelFormat.writeS( pixelFormat );
                    }
                    if( !acquisitionMode.empty() )
                    {
                        ac.acquisitionMode.writeS( acquisitionMode );
                    }
                    acquisitionMode = ac.acquisitionMode.readS();
                    cout << "Device set up to " << ifc.pixelFormat.readS() << " " << ifc.width.read() << "x" << ifc.height.read() << endl;
                }
                break;
            case dilDeviceSpecific:
                {
                    CameraSettingsBase cs( pDev );
                    if( width > 0 )
                    {
                        cs.aoiWidth.write( width );
                    }
                    if( height > 0 )
                    {
                        cs.aoiHeight.write( height );
                    }
                    cout << "Device set up to " << cs.aoiWidth.read() << "x" << cs.aoiHeight.read() << endl;
                }
                break;
            default:
                break;
            }
        }
        catch( ImpactAcquireException& e )
        {
            // this e.g. might happen if the same device is already opened in another process...
            cout << "*** " << __FUNCTION__ << " - An error occurred while opening the device " << pDev->serial.read()
                 << "(error code: " << e.getErrorCode() << ", " << e.getErrorCodeAsString() << "). Press any key to end the application..." << endl;
            PRESS_A_KEY_AND_RETURN
        }

        // start the execution of the 'live' loop.
        liveLoop( pDev, boStoreFrames, settingName, acquisitionMode == "SingleFrame" );
        cout << " == Will exit...." << endl;
        pDev->close();
        // do NOT delete pDev here! It will be destroyed automatically when the device manager (devMgr) is destroyed.
        // There is also no real need to close the device for the same reasons. Once the last instance of 'DeviceManager'
        // objects moves out of scope all devices open in the current process context will be closed automatically.
    }
    else
    {
        cout << "Unable to continue!";
    }
    cout << " -- ending application...." << endl;

    return 0;
}
