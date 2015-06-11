#ifndef TimeoutH
#define TimeoutH TimeoutH

#include <time.h>
#include <wx/utils.h>

class CTimeout
{
public:
	explicit CTimeout( unsigned long timeoutmsec )
				{
					Start( timeoutmsec );
				}
	~CTimeout( ){ ; }
	void Start( unsigned long timeoutmsec )
	{
		start = clock( );
		elapse_period = timeoutmsec * CLOCKS_PER_SEC / 1000;
		if( elapse_period == 0 )
			elapse_period = 1;
		end = start + elapse_period;
	}

	unsigned char Elapsed( )
	{
		return ( end < clock() );
	}

	void WaitTimeout( unsigned long milliseconds )
	{
		wxMilliSleep( milliseconds );
	}

	unsigned long Remain( )
	{
		if( Elapsed( ) )
			return 0;
		else
			return ((clock() - end) * 1000) / CLOCKS_PER_SEC;
	}

private:
	clock_t elapse_period, start, end;
};

#endif //TimeoutH
