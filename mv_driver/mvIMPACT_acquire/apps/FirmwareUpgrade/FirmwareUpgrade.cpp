//-----------------------------------------------------------------------------
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
void printHelp( void )
//-----------------------------------------------------------------------------
{
	cout << " Available commands / usage:" << endl
		<< " ------------------------------------" << endl
		<< " no parameters: silent mode (will update the firmware for every mvBlueFOX connected to the system" << endl
		<< "     without asking any further questions" << endl
		<< endl
		<< " -d<serial>: will update the firmware for a device with the serial number specified by" << endl
		<< "             <serial>. Pass '*' as a wildcard" << endl
		<< "             EXAMPLE: FirmwareUpgrade -dBF* // will update the firmware for every device with a serial number starting with 'BF'" << endl
		<< endl
		<< " -sel: Will prompt the user to select a device to update" << endl
		<< endl
		<< " -help: Will print this help." << endl;
}

//-----------------------------------------------------------------------------
void updateFirmware( Device* pDev )
//-----------------------------------------------------------------------------
{
	if( pDev )
	{
		if( pDev->family.read() == "mvBlueFOX" )
		{
			cout << "The firmware of device " << pDev->serial.read() << " is currently " << pDev->firmwareVersion.readS() << "." << endl;
			cout << "It will now be updated. During this time(approx. 30 sec.) the application will not react. Please be patient." << endl;
			int result = pDev->updateFirmware();
			if( result == DMR_FEATURE_NOT_AVAILABLE )
			{
				cout << "This device doesn't support firmware updates." << endl;
			}
			else if( result != DMR_NO_ERROR )
			{
				cout << "An error occurred: " << ImpactAcquireException::getErrorCodeAsString( result ) << ". (please refer to the manual for this error code)." << endl;
			}
			else
			{
				cout << "Firmware update done. Result: " << pDev->HWUpdateResult.readS() << endl;
				if( pDev->HWUpdateResult.read() == urUpdateFWOK )
				{
					cout << "Update successful." << endl;
					if( pDev->family.read() == "mvBlueFOX" )
					{
						cout << "Please disconnect and reconnect the device now to activate the new firmware." << endl;
					}
				}
			}
		}
		else
		{
			cout << "*** Error: This application is meant for mvBlueFOX devices only. This is an " << pDev->family.read() << " device." << endl;
		}
	}
	else
	{
		cout << "*** Error: Invalid device pointer passed to update function." << endl;
	}
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[])
//-----------------------------------------------------------------------------
{
	if( argc > 2 )
	{
		cout << "Invalid input parameter count" << endl
			<< endl;
		printHelp();
		return 0;
	}

	unsigned int devCnt = 0;
	DeviceManager devMgr;
	if(( devCnt = devMgr.deviceCount()) == 0 )
	{
		cout << "*** Error: No MATRIX VISION device found! Unable to continue!" << endl;
		return 0;
	}

	cout << "Have found " << devCnt << " devices on this platform!" << endl
		<< "Please note that this application will only work for mvBlueFOX devices" <<endl;

	if( argc == 1 )
	{
		int index = 0;
		Device* pDev = 0;
		while( ( pDev = devMgr.getDeviceByFamily( "mvBlueFOX", index ) ) != 0 )
		{
			updateFirmware( pDev );
			++index;
		}
	}
	else
	{
		string command(argv[1]);
		if( command == "-sel" )
		{
			Device* pDev = getDeviceFromUserInput( devMgr );
			if( !pDev )
			{
				return 0;
			}
			updateFirmware( pDev );
		}
		else if( command.find( "-d" ) == 0 )
		{
			string serial(command.substr( 2 ));
			Device* pDev = devMgr.getDeviceBySerial( serial );
			if( !pDev )
			{
				cout << "Can't find Device " << serial << endl;
				return 0;
			}
			updateFirmware( pDev );
		}
		else if( command == "-help" )
		{
			printHelp();
		}
		else
		{
			cout << "Invalid input parameter: " << command << endl
				<< endl;
			printHelp();
			return 0;
		}
	}

	return 0;
}
