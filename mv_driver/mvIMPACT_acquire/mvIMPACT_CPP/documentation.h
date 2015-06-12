/*!
\mainpage Introduction to the C++ interface reference

\tableofcontents

\section Overview Overview
This is the documentation for developers who want to work with the C++ interface of mvIMPACT Acquire. It is based on the C interface but provides a more convenient and object orientated approach to properties and functions offered by a device driver.

The complete C++ part of the interface is provided in source. This has some advantages:

- This interface works for every C++ compiler
- The C++ wrapper might act as a source of information for programmers coming from other programming languages (e.g. Delphi) when writing their own wrappers
- The user can step through this part of the code

Every program written using this interface will start in one or the other form with creating an instance of the class \b mvIMPACT::acquire::DeviceManager. Each application needs at least one instance of this class while devices shall be accessed. To find out how to gain access to a certain device look at the detailed description of this class.

Once a pointer (or in other words: access) to the desired device represented by an instance of the class \b mvIMPACT::acquire::Device has been obtained every other device related properties or functions can be accessed.

<div style="padding:5px; border:2px dashed #005288">Stuff from the mvIMPACT::acquire::GenICam namespace is only available when using the mvIMPACT::acquire::dilGenICam interface layout (by setting mvIMPACT::acquire::Device::interfaceLayout to mvIMPACT::acquire::dilGenICam before opening the device. Classes from the mvIMPACT::acquire namespace might only be available in interface layout mvIMPACT::acquire::dilDeviceSpecific. These classes will state their limited availability in the documentation. Without setting the interface layout, mvIMPACT::acquire::dilDeviceSpecific will be used by default (with the exception of mvBlueLYNX-X; here mvIMPACT::acquire::dilGenICam will be used by default). The example \ref GenICamInterfaceLayout.cpp shows how to set the interface layout.</div>

\note
When the last instance to objects of the class \b mvIMPACT::acquire::DeviceManager is destroyed, every device pointer or handle to interface properties will become invalid automatically, as the destructor of the device manager decrements an internal usage counter, that automatically closes all devices and frees allocated resources, once this usage counter reaches 0. So make sure there is always at least one instance of the device manager present in your application.

Some source code samples how to locate a certain \b mvIMPACT::acquire::Device also can be found in the detailed description of the class \b mvIMPACT::acquire::DeviceManager.

\section mvIMPACTImageProcessing mvIMPACT image processing

When additional image processing of the captured images is required as well the mvIMPACT image processing library can be used in connection with this capture interface. All image processing functions however will ask for a special image buffer format. The device can deliver this format natively but the functions returning this data format will not by default be included into the interface thus allowing the usage of the capture interface only without the need to install the complete image processing library as well. To include the functions that will return mvIMPACT image processing library compliant buffers to the user application the main header of the library must be included \b BEFORE including the capture interface headers:

\code
#include <mvIMPACT.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
\endcode

If this include order is used, the class \b mvIMPACT::acquire::Request will provide 2 additional versions of the method \b mvIMPACT::acquire::Request::getIMPACTImage that can be used to obtain images in an image processing library compatible format.

\section GettingStarted Getting started

To capture an image an instance of the class \b mvIMPACT::acquire::FunctionInterface must be created. This class can be constructed by passing a pointer to the \b mvIMPACT::acquire::Device object obtained from the \b mvIMPACT::acquire::DeviceManager to the class constructor.

The function interface class provides access to most of the devices executable functions, while most of the settings (e.g. the exposure time or the trigger mode) are implemented as properties (see e.g. \b mvIMPACT::acquire::Property for details).

Getting the first image might e.g. look like that:

\code
//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
//-----------------------------------------------------------------------------
{
  mvIMPACT::acquire::DeviceManager devMgr;
  if( devMgr.deviceCount() == 0 )
  {
    cout << "No device found! Unable to continue!" << endl;
    char ch = getch();
    return 0;
  }

  cout << "Initialising the device. This might take some time..." << endl;
  // create an interface to the first device found
  mvIMPACT::acquire::Device* pDev = devMgr[0];
  try
  {
    pDev->open();
  }
  catch(mvIMPACT::acquire::ImpactAcquireException& e )
  {
    // this e.g. might happen if the same device is already opened in another process...
    cout << "An error occurred while opening the device(error code: " << e.errCode()
         << "). Press any key to end the application..." << endl;
    char ch = getch();
    return 0;
  }

  mvIMPACT::acquire::FunctionInterface fi( pDev );

  // send a request to the default request queue of the device and wait for the result.
  fi.imageRequestSingle();
  // Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
  if( pThreadParameter->pDev->acquisitionStartStopBehaviour.read() == assbUser )
  {
    TDMR_ERROR result = DMR_NO_ERROR;
    if( ( result = static_cast<TDMR_ERROR>(fi.acquisitionStart()) ) != DMR_NO_ERROR )
    {
      cout << "'FunctionInterface.acquisitionStart' returned with an unexpected result: " << result
          << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
    }
  }
  // wait for results from the default capture queue
  int requestNr = fi.imageRequestWaitFor( -1 );

  // check if the image has been captured without any problems
  if( !fi.isRequestNrValid( requestNr ) )
  {
    // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
    // additional information under TDMR_ERROR in the interface reference
    cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")"
         << ", timeout value too small?" << endl;
  }

  const mvIMPACT::acquire::Request* pRequest = fi.getRequest(requestNr);
  if( !pRequest->isOK() )
  {
    cout << "ERROR! Request result: " << pRequest->requestResult.readS() << endl;
    return 0;
  }

  // everything went well. Do whatever you like with the result
  const int width = pRequest->imageWidth.read();
  // unlock the buffer to let the driver know that you no longer need this buffer
  fi.imageRequestUnlock( requestNr );
  return 0;
}
\endcode

This sample contains everything the user needs to do to capture one image including all initialization work and error handling for every source of error one can think of.

Several sample applications will provide an even better understanding of the interface.

\section WorkingWithProperties Working with properties

As mentioned before, most of the settings available are implemented as properties rather than as functions. A growing collection of classes provide access to properties, which can be read and some of them set by the user. All objects which grant access to device driver interface properties will require the pointer to the \b mvIMPACT::acquire::Device acquired from a \b mvIMPACT::acquire::DeviceManager object again.

\note
It is not necessary to call \b mvIMPACT::acquire::Device::open before creating objects requiring a pointer to a device object, but each constructor accepting such a pointer will need the device to be opened, so the first object constructed from a pointer to a closed \b mvIMPACT::acquire::Device object will try to initialize the device, which is why the first constructor call might take some time.

\note
If for some reason the device can't be initialized, an exception might be thrown by any of those constructors.

Objects the user can create to modify or read \b mvIMPACT::acquire::Property values include (among other):

\subsection WorkingWithProperties_Basic Basic objects

<table width="100%">
<tr>
  <th class="indexkey">Name of the class</th>
  <th class="indexkey">Description</th>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::BasicDeviceSettings </td>
  <td class="indexvalue">A collection of basic settings. Please note that device specific classes derived from this class might offer more features</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::ImageProcessing </td>
  <td class="indexvalue">Various properties and methods to process the image before transmitted to the user</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::IOSubSystem </td>
  <td class="indexvalue">Provides access to the digital I/Os of the device and to real time control machines if available</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::ImageDestination </td>
  <td class="indexvalue">Properties to control the destination format of the image</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::Info </td>
  <td class="indexvalue">General information about the device and the driver</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::Statistics </td>
  <td class="indexvalue">Provides access to statistical information like the current frames per second</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::SystemSettings </td>
  <td class="indexvalue">Provides access to settings controlling the overall behaviour of the driver</td>
</tr>
</table>

\subsection WorkingWithProperties_BC mvBlueCOUGAR

<table width="100%">
<tr>
  <th class="indexkey">Name of the class</th>
  <th class="indexkey">Description</th>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::CameraSettingsBlueCOUGAR</td>
  <td class="indexvalue">A collection of \b mvBlueCOUGAR and \b mvBlueLYNX-M7 specific settings</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::Connector</td>
  <td class="indexvalue">Properties to control the video input channel selection for \b mvBlueLYNX-M7 devices</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::IOSubSystemBlueCOUGAR</td>
  <td class="indexvalue">Provides access to the digital I/Os of the device and to real time control machines if available</td>
</tr>
</table>

\subsection WorkingWithProperties_BF mvBlueFOX

<table width="100%">
<tr>
  <th class="indexkey">Name of the class</th>
  <th class="indexkey">Description</th>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::CameraSettingsBlueFOX</td>
  <td class="indexvalue">A collection of \b mvBlueFOX specific settings</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::SettingsBlueFOX</td>
  <td class="indexvalue">A combination of the three classes \b mvIMPACT::acquire::SettingsBlueFOX, \b mvIMPACT::acquire::ImageDestination and \b mvIMPACT::acquire::ImageProcessing</td></td>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::InfoBlueFOX </td>
  <td class="indexvalue">Specific information about the \b mvBlueFOX and the driver</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::IOSubSystemBlueFOX </td>
  <td class="indexvalue">Provides access to the digital I/Os of the device and to real time control machines if available</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::SystemBlueFOX </td>
  <td class="indexvalue">Provides access to settings controlling the overall behaviour of the driver</td>
</tr>
</table>

\subsection WorkingWithProperties_FG Frame Grabbers

<table width="100%">
<tr>
  <th class="indexkey">Name of the class</th>
  <th class="indexkey">Description</th>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::CameraSettingsFrameGrabber </td>
  <td class="indexvalue">A collection of device specific settings</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::Connector</td>
  <td class="indexvalue">Properties to control the video input channel selection</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::SettingsFrameGrabber </td>
  <td class="indexvalue">A combination of the four classes \b mvIMPACT::acquire::CameraSettingsFrameGrabber, \b mvIMPACT::acquire::Connector, \b mvIMPACT::acquire::ImageDestination and \b mvIMPACT::acquire::ImageProcessing</td></td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::IOSubSystemFrameGrabber </td>
  <td class="indexvalue">Provides access to the digital I/Os of the device and to real time control machines if available</td>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber</td>
  <td class="indexvalue">Provides access to high level functions to control the creation of complex signals on digital outputs</td>
</tr>
</table>

\subsection WorkingWithProperties_VD mvVirtualDevice

<table width="100%">
<tr>
  <th class="indexkey">Name of the class</th>
  <th class="indexkey">Description</th>
</tr>
<tr>
  <td class="indexkey">\b mvIMPACT::acquire::CameraSettingsVirtualDevice</td>
  <td class="indexvalue">A collection of \b mvVirtualDevice specific settings</td>
</tr>
</table>

\subsection WorkingWithProperties_GenICam GenICam interface layout

Please refer to the \ref ImageAcquisition_section_genicam_acessingProperties section for details.
*/

/*!
\page Working_with_camera_descriptions Working with camera descriptions (frame grabbers only)

\tableofcontents

Certain capture device (e.g. frame grabber) can process data from a wide range of imaging devices (e.g. cameras). However in order to interpret the incoming data from an imaging device correctly, the capture device needs to be given a certain amount of information about the structure of the video signal.

The \b mvIMPACT \b Acquire interface addresses this necessity by the introduction of so called \a "camera descriptions". A \a "camera description" is a certain set of parameters that should enable the capture device to cope with the incoming image data to reconstruct a correct image from the imaging device in the memory of the host system. For instance this information may contain information whether the image is transmitted as a whole or if it's transmitted as individual blocks (e.g. when dealing with interlaced cameras) that need to be reconstructed in a certain way to form the complete image.

Each capture device will support different sets of parameters. For example some capture devices will only be able to capture image data from standard video source such as a \b PAL or \b NTSC compliant camera, while others might only be capable to acquire data from digital image sources such as \b CameraLink compliant cameras. To reflect these device specific capabilities the \a "camera descriptions" have been grouped into different base classes. See \b mvIMPACT::acquire::CameraDescriptionBase and derived classes to find out how the basic structure of these objects look. Which basic \a "camera description" classes are supported by an individual device can be seen directly after the device has been initialized by creating and instance of \b mvIMPACT::acquire::CameraDescriptionManager. These objects will contain different lists for each camera class. One or more of this lists might be empty for a device, which indicates, that this device doesn't support this particular group of descriptions.

\note
For devices that don't support camera descriptions at all, creating an instance of \b mvIMPACT::acquire::CameraDescriptionManager will raise an exception, that should be handled appropriately.

To select a certain camera description to be used to prepare the capture device for the expected data the property \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type can be modified. Its translation dictionary (e.g. see \b mvIMPACT::acquire::EnumPropertyI::getTranslationDictString) will contain every camera description currently available.

\section CreateANewCameraDescription Create a new camera description

Now when a camera is connected, that differs in one or more parameter(s) from the default offered by one of the available base classes and no special description for the imaging device in question is available a new matching description must be generated.

\note
Its also possible to modify one of the standard descriptions to adapt the parameter set to the used imaging device, but this method is not recommend as this would define something to be 'standard', which in fact is not. Therefore it is not possible to store the standard descriptions permanently. It is however possible to modify and work with the changed parameters, but these changes will be lost once the device is closed.

The recommended way of adapting an imaging source to a capture device is to create a new description for a imaging device that does not completely fall into one of the offered standard descriptions. The first thing to decide when creating a new camera description is to which existing description offers the closest match for the new description. Once this has been decided a copy of this description can be created with an arbitrary name(that must be unique within the family the description is created from).

\note
For an example how to create a new camera description see the description of \b mvIMPACT::acquire::CameraDescriptionBase.

Afterwards the newly created camera description will be added to the list of existing ones. It will therefore be available via the instance of \b mvIMPACT::acquire::CameraDescriptionManager and also selectable via \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type It's parameters at this point will match the \a "parent" description(the one the function \b mvIMPACT::acquire::CameraDescriptionBase::copyDescription was executed from) completely.

Now the reason for creating a new camera description was that the parameters in the existing description didn't exactly match the connected imaging device. Therefore the next step would probably be to modify some of the parameters.

\section StoringCameraDescriptions Storing camera descriptions

\note
A new camera description will \b NOT be stored permanently by default. In order to make this description available the next time the capture device is initialized, the newly created description must be exported via a function call.

To store a camera description permanently the function \b mvIMPACT::acquire::CameraDescriptionBase::exportDescription of the new camera description must be invoked.

As a direct result the modified settings will become the new default values of this particular camera description.

\note
Please note, that this will \b NOT work for one of the standard camera descriptions. Whenever the user tries to export one of these, the error \b mvIMPACT::acquire::DMR_EXECUTION_PROHIBITED will be returned. This is to reflect the fact, that a standard can't be manually modified. This must \b ALWAYS be done by creating a new description.

When exporting a camera description a file in XML format will be written to disc. Under Windows camera descriptions will be stored under \b "%ALLUSERS%\Documents\MATRIX VISION\mvIMPACT Acquire\CameraFiles" (or \b "%MVIMPACT_ACQUIRE_DATA_DIR%\CameraFiles" which will point to the same folder), under Linux this directory will be \b "/etc/matrix-vision/mvimpact-acquire/camerafiles" while under other platforms these files will end up in the current working directory.

Now when closing and re-opening a device only the default camera descriptions an the one selected before settings have been saved will appear in the list of camera descriptions of an instance of \b mvIMPACT::acquire::CameraDescriptionManager. This is to save memory. However all detected camera descriptions will be available via the property \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type

Once a description is selected, that hasn't been in the list of camera descriptions before, it will be created and thus will become available for modifications again via the class \b mvIMPACT::acquire::CameraDescriptionManager.

Again: For a different camera a new description should be generated, to operate complex cameras in different modes, either a new description can be generated or an existing one can be modified.

After a camera has been modified the function \b mvIMPACT::acquire::CameraDescriptionBase::importDescription can be used to fall back to the values stored in the camera description file. This will restore the default settings for this description. The function \b mvIMPACT::acquire::ComponentCollection::restoreDefault does serve the same purpose, but will work for default descriptions as well.

\section PropertyInteraction Property interaction

Certain properties will affect other properties depending on the value written to the property. This section is meant to help to answer some of the questions, that might arise from this behaviour.

\b mvIMPACT::acquire::CameraSettingsFrameGrabber::acquisitionField <-> \b mvIMPACT::acquire::CameraDescriptionStandardBase::startField:

While \b mvIMPACT::acquire::CameraSettingsFrameGrabber::acquisitionField is set to \b mvIMPACT::acquire::afAuto the field selected in the property \b mvIMPACT::acquire::CameraDescriptionStandardBase::startField will be used to trigger the acquisition.

\b mvIMPACT::acquire::CameraSettingsFrameGrabber::interlacedMode <-> \b mvIMPACT::acquire::CameraDescriptionNonStandard::interlacedType:

When the latter property is set to \b mvIMPACT::acquire::citNone the property \b mvIMPACT::acquire::CameraSettingsFrameGrabber::interlacedMode will be invisible, as it doesn't make sense to define how an interlaced signal has to be reconstructed if no interlaced signal is present.

If \b mvIMPACT::acquire::CameraDescriptionNonStandard::interlacedType is set either to \b mvIMPACT::acquire::citInterlaced or \b mvIMPACT::acquire::citInvertedInterlaced, \b mvIMPACT::acquire::CameraSettingsFrameGrabber::interlacedMode can be used to define whether the resulting image shall be reconstructed from the odd and even frame of the interlaced signal (\b mvIMPACT::acquire::CameraSettingsFrameGrabber::interlacedMode then must be set to \b mvIMPACT::acquire::imOn) or if the single fields shall be treated as individual images (\b mvIMPACT::acquire::CameraSettingsFrameGrabber::interlacedMode then must be set to \b mvIMPACT::acquire::imOff).

In the latter situation either just one particular field (either odd or even) or every field can be captured. This again can be defined via the two properties \b mvIMPACT::acquire::CameraSettingsFrameGrabber::acquisitionField and \b mvIMPACT::acquire::CameraDescriptionStandardBase::startField. The following table will show the behaviour for a camera signal depending on these selections:

\htmlonly
<table width="100%">
<tr>
  <td class="header" width="20%">Value of &quot;interlacedType&quot;</td>
  <td class="header" width="10%">Value of &quot;interlacedMode&quot;</td>
  <td class="header" width="20%">Value of &quot;acquisitionField&quot;</td>
  <td class="header" width="10%">Value of &quot;startField&quot;</td>
  <td class="header" width="40%">Result of this setting / behaviour</td>
</tr>
<tr>
  <td class="indexvalue">citNone</td>
  <td class="indexvalue">no influence(will be invisible)</td>
  <td class="indexvalue">no influence(will be invisible)</td>
  <td class="indexvalue">no influence</td>
  <td class="indexvalue">\b interlaced \b video \b sources: \n all fields will be treated like individual images no matter whether it's an odd or even field. \n  \n \b non-interlaced \b video \b sources: \n Here we are just dealing with full frames thus every frame will be captured.</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOn</td>
  <td class="indexvalue">no influence(will be invisible)</td>
  <td class="indexvalue">afEven</td>
  <td class="indexvalue">full images merged from one even and one odd field will be captured. The acquisition will start with the next detected even field after an image has been requested.</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOn</td>
  <td class="indexvalue">no influence(will be invisible)</td>
  <td class="indexvalue">afOdd</td>
  <td class="indexvalue">full images merged from one even and one odd field will be captured. The acquisition will start with the next detected odd field after an image has been requested.</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOn</td>
  <td class="indexvalue">no influence(will be invisible)</td>
  <td class="indexvalue">afAny</td>
  <td class="indexvalue">full images merged from one even and one odd field will be captured. The acquisition will start with the next detected field after an image has been requested.</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOff</td>
  <td class="indexvalue">afAuto (the value of &quot;startField&quot; will be used)</td>
  <td class="indexvalue">afEven</td>
  <td class="indexvalue" rowspan="2">Only even fields will be captured. These will be treated as individual images.</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOff</td>
  <td class="indexvalue">afEven</td>
  <td class="indexvalue">no influence</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOff</td>
  <td class="indexvalue">afAuto (the value of &quot;startField&quot; will be used)</td>
  <td class="indexvalue">afOdd</td>
  <td class="indexvalue" rowspan="2">Only odd fields will be captured. These will be treated as individual images.</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOff</td>
  <td class="indexvalue">afOdd</td>
  <td class="indexvalue">no influence</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOff</td>
  <td class="indexvalue">afAuto (the value of &quot;startField&quot; will be used)</td>
  <td class="indexvalue">afAny</td>
  <td class="indexvalue" rowspan="2">all fields will be captured and treated like individual images (alternating odd and even).</td>
</tr>
<tr>
  <td class="indexvalue">citInterlaced or citInvertedInterlaced</td>
  <td class="indexvalue">imOff</td>
  <td class="indexvalue">afAny</td>
  <td class="indexvalue">afAny</td>
</tr>
</table>
\endhtmlonly

When dealing with CameraLink camera descriptions there are some other dependencies between CameraLink specific properties.

Dependency between \b mvIMPACT::acquire::CameraDescriptionCameraLink::bitsPerPixel and \b mvIMPACT::acquire::CameraDescriptionCameraLink::pixelsPerCycle:

\htmlonly
<table width="100%">
<tr>
  <td class="header" width="50%">Value of &quot;BitsPerPixel&quot;</td>
  <td class="header" width="50%">Allowed values for &quot;PixelsPerCycle&quot;</td>
</tr>
<tr>
  <td class="indexvalue">8</td>
  <td class="indexvalue">1, 2, 3, 4, 8</td>
</tr>
<tr>
  <td class="indexvalue">10</td>
  <td class="indexvalue" rowspan="2">1, 2, 3, 4</td>
</tr>
<tr>
  <td class="indexvalue">12</td>
</tr>
<tr>
  <td class="indexvalue">14</td>
  <td class="indexvalue" rowspan="5">1</td>
</tr>
<tr>
  <td class="indexvalue">16</td>
</tr>
<tr>
  <td class="indexvalue">24</td>
</tr>
<tr>
  <td class="indexvalue">30</td>
</tr>
<tr>
  <td class="indexvalue">36</td>
</tr>
</table>
\endhtmlonly

Dependency between \b mvIMPACT::acquire::CameraDescriptionCameraLink::pixelsPerCycle, \b mvIMPACT::acquire::CameraDescriptionCameraLink::tapsXGeometry and \b mvIMPACT::acquire::CameraDescriptionCameraLink::tapsYGeometry:

\htmlonly
<table width="100%">
<tr>
  <td class="header" width="20%">Value of &quot;PixelsPerCycle&quot;</td>
  <td class="header" width="40%">Allowed values for &quot;TapsXGeometry&quot;</td>
  <td class="header" width="40%">Allowed values for &quot;TapsYGeometry&quot;</td>
</tr>
<tr>
  <td class="indexvalue">1</td>
  <td class="indexvalue">cltxg1X</td>
  <td class="indexvalue">cltyg1Y</td>
</tr>
<tr>
  <td class="indexvalue" rowspan="2">2</td>
  <td class="indexvalue">cltxg1X</td>
  <td class="indexvalue">cltyg1Y2, cltyg2YE</td>
</tr>
<tr>
  <td class="indexvalue">cltxg1X2, cltxg2X, cltxg2XE, cltxg2XM</td>
  <td class="indexvalue">cltyg1Y</td>
</tr>
<tr>
  <td class="indexvalue">3</td>
  <td class="indexvalue">cltxg1X3, cltxg3X</td>
  <td class="indexvalue">cltyg1Y</td>
</tr>
<tr>
  <td class="indexvalue" rowspan="2">4</td>
  <td class="indexvalue">cltxg1X4, cltxg4X, cltxg2X2, cltxg2X2E, cltxg2X2M, cltxg4XE</td>
  <td class="indexvalue">cltyg1Y</td>
</tr>
<tr>
  <td class="indexvalue">cltxg1X2, cltxg2X, cltxg2XE, cltxg2XM</td>
  <td class="indexvalue">cltyg1Y2, cltyg2YE</td>
</tr>
<tr>
  <td class="indexvalue">8</td>
  <td class="indexvalue">cltxg1X8, cltxg8X</td>
  <td class="indexvalue">cltyg1Y</td>
</tr>
</table>
\endhtmlonly
*/
