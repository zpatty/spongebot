
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
//#include <time.h>
//#include <sys/time.h>

#include "RFT_UART_SAMPLE.h"

BYTE port = 0;

DWORD buad = B115200;

//DWORD buad = B921600;

DWORD byte_size = CS8;

CRT_RFT_UART RFT_SENSOR;

float		g_force_divider 	= 50.0f;			// for more information to refer the RFT sensor manual
float		g_torque_divider 	= 2000.0f;			// for more information to refer the RFT sensor manual

int main()
{
	bool isGo = true;

	char devName[] = "/dev/ttyUSB";

	printf("RFT TEST SOFWARE\n");

	RFT_SENSOR.openPort( devName, port, buad, byte_size );
	// initialize force/torque divider
	RFT_SENSOR.m_RFT_IF_PACKET.setDivider(g_force_divider, g_torque_divider); // V1.1
	usleep(1000000);
	
	while( isGo )
	{
		char cmd = getchar();
		
		if( cmd == 0x1B ) // esc
			isGo = false;
		else if( cmd == 'M' || cmd == 'm' ) // measure
			RFT_SENSOR.rqst_FT_Continuous();
		else if( cmd == 'S' || cmd == 's' ) // stop
			RFT_SENSOR.rqst_FT_Stop();
		else if( cmd == 'B' || cmd == 'b' ) // bias
			RFT_SENSOR.set_FT_Bias(1);
		else if( cmd == 'U' || cmd == 'u' ) // un-bias
			RFT_SENSOR.set_FT_Bias(0);
		else
		{
			if( cmd != '\r' && cmd != '\n' )
				printf("UNKNOWN COMMAND\n");
		}
	}


	RFT_SENSOR.closePort();
	
	return 0;
}

///////////////////////
// End of This File
