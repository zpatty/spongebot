
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
//#include <time.h>
//#include <sys/time.h>

#include "RFT_UART_SAMPLE.h"

#include "RFT_IF_PACKET_Rev1.2.h"

#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::tcp;
using std::string;
using std::cout;
using std::endl;

BYTE port = 1;

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
	
	// RFT_SENSOR.client(9,"127.0.0.1");

	

    //connect to host
    RFT_SENSOR.client.conn("127.0.0.1" , 10000);

    //send some data

	// RFT_SENSOR.client.send_data("testing from client");
	
 // // getting response from server
 //    boost::asio::streambuf receive_buffer;
 //    boost::asio::read(socket, receive_buffer, boost::asio::transfer_all(), error);
 //    if( error && error != boost::asio::error::eof ) {
 //        cout << "receive failed: " << error.message() << endl;
 //    }
 //    else {
 //        const char* data = boost::asio::buffer_cast<const char*>(receive_buffer.data());
 //        cout << data << endl;
 //    }

	// usleep(1000);
	
	while( isGo )
	{
		char cmd = getchar();
		
		if( cmd == 0x1B ) // esc
			isGo = false;
		else if( cmd == 'M' || cmd == 'm' )
		{ // measure
			RFT_SENSOR.rqst_FT_Continuous();
			// char buf[100];
			// int return_Val;
			// sprintf(buf, "%0.3f, %.03f, %.03f, %0.3f, %.03f, %.03f\r",
			// RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[0], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[1], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[2],
			// RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[3], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[4], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[5] );
   //   		boost::system::error_code error;
   //   		boost::asio::write( socket, boost::asio::buffer(buf), error );
     	}
		else if( cmd == 'S' || cmd == 's' ) // stop
			RFT_SENSOR.rqst_FT_Stop();
		else if( cmd == 'B' || cmd == 'b' ) // bias
			RFT_SENSOR.set_FT_Bias(1);
		else if( cmd == 'U' || cmd == 'u' ) // un-bias
			RFT_SENSOR.set_FT_Bias(0);
		else if( cmd == 'F' || cmd == 'f' ) // un-bias
			RFT_SENSOR.set_FT_Filter_Type(1, 4);
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
