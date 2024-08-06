#include "WinTypes.h"

#include <unistd.h>
#include <sys/types.h>
//#include <stdio.h>
#include <fcntl.h>   /* File control definitions */
//#include <errno.h>   /* Error number definitions */
#include <termios.h>
#include <pthread.h>

#include <iostream>
#include <boost/asio.hpp>

#include <cstdlib>
#include <deque>
#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp>

using boost::asio::ip::tcp;


using namespace boost::asio;
using ip::tcp;
using std::string;
using std::cout;
using std::endl;

#include <vector>
using namespace std;

#ifndef _RT_RS232_H_
#define _RT_RS232_H_



#include "RFT_IF_PACKET_Rev1.2.h" // RFT F/T SENSOR DATA FIELD & UART PACKET HANDLING SOURCE

/*
see termbits.h file
#define  B0	  
#define  B50	
#define  B75	
#define  B110	
#define  B134	
#define  B150	
#define  B200	
#define  B300	
#define  B600	
#define  B1200
#define  B1800
#define  B2400
#define  B4800
#define  B9600
#define  B19200
#define  B38400
#define  B57600  
#define  B115200 
#define  B230400 
#define  B460800 
#define  B500000 
#define  B576000 
#define  B921600 
#define  B1000000
#define  B1152000
#define  B1500000
#define  B2000000
#define  B2500000
#define  B3000000
#define  B3500000
#define  B4000000

#define CSIZE
#define CS5
#define CS6
#define CS7
#define CS8
#define CSTOPB
#define CREAD	
#define PARENB
#define PARODD
#define HUPCL	
#define CLOCAL
#define CBAUDEX

*/
// Flow control flags
#define FC_DTRDSR       0x01
#define FC_RTSCTS       0x02
/* REMOVE Software Flow Control
#define FC_XONXOFF      0x04
*/
#define FC_NONE         0x00

// ascii definitions
#define ASCII_BEL       0x07
#define ASCII_BS        0x08
#define ASCII_LF        0x0A
#define ASCII_CR        0x0D
#define	ASCII_XON		    0x11
#define	ASCII_XOFF		  0x13

#define RFT_UART_TX_BUFF_SIZE (1024)
#define RFT_UART_RX_BUFF_SIZE (1024*4)



class CRT_RFT_UART
{

public:
	CRT_RFT_UART( void );
	~CRT_RFT_UART( void );

	// EXTERNAL INTERFACE FUNCTIONS
	BOOL	openPort( char *devName, BYTE bPort = 0, DWORD rate = B115200, DWORD bByteSize = CS8 );
	BOOL	closePort( void );
	BOOL  isConnected( void );


	BOOL writeBuffer( BYTE* lpBuffer, DWORD dwBytesToWrite, LPDWORD lpdwBytesWritten );

	void	readWorker( void );         // read worker.

	/*
    TCP Client class
*/
	class TcpClient
	{
		private:
			int sock;
			std::string address;
			string response_data = "";
			int port;
			struct sockaddr_in server;

		public:
			TcpClient();
			bool conn(string, int);
			bool send_data(string data);
			string receive(int);
	};

	TcpClient client;

	
	////////////////////////////////////////////////////////////////////////////////////////////
	// FOR RFT INTERFACE
	CRT_RFT_IF_PACKET m_RFT_IF_PACKET;	// data field & uart packet handling class
	
	bool rqst_ProductName(void);		// read product name
	bool rqst_SerialNumber(void);		// read serial number
	bool rqst_Firmwareverion(void);		// read firmware version
	bool rqst_CommSpeed(void);			// read baud-rate
	bool rqst_FT_Filter_Type(void);		// read filter type
	bool rqst_FT(void);					// read force/torque (once)
	bool rqst_FT_Continuous(void);		// start force/torque continuous output 
	bool rqst_FT_Stop(void);			// stop force/torque continuous output
	bool rqst_FT_Cont_Interval(void);   // read force/torque output frq.
	bool rqst_FT_OverloadCnt(void);		// read overload count

	bool set_Comm_Speed(int comm_speed);						// set baud-rate
	bool set_FT_Filter_Type(int filter_type, int sub_type);		// set filter
	bool set_FT_Cont_Interval(int interval);					// set force/torque output frq.
	bool set_FT_Bias(int is_on);								// set bias

	int m_nCurrMode;					// current operation mode or command type
	bool m_bIsRcvd_Response_Pkt;		// receive flag for response packet of current command

	void RFT_Data_Handler(void);		// receive packet handler

	void rcvd_ProduectName(void);		// check function for response of read product name command
	void rcvd_SerialNumber(void);		// check function for response of read serial number command
	void rcvd_Firmwareversion(void);	// check function for response of read firmware version command
	void rcvd_CommSpeed(void);			// check function for response of read baud-rate command
	void rcvd_FT_Filter_Type(void);		// check function for response of read filter type command
	void rcvd_FT(void);					// check function for response of read force/torque command
	void rcvd_FT_Cont_Interval(void);	// check function for response of read force/torque output frq. command
	void rcvd_FT_Overload_Count(void);	// check function for response of read overload count command

	void rcvd_Response_Set_Comm_Speed(void);		// check function for response of set baud-rate command
	void rcvd_Response_Set_FT_Filter_Type(void);	// check function for response of set filter type command
	void rcvd_Response_Set_FT_Cont_Interval(void);	// check function for response of set force/torque output frq. command
  
protected:
	void initialize_variables(void);

	BOOL		  m_bConnected;		        // Open/Closed indicating flag
  
 	int m_nRcvdBufferIdx;
	unsigned char m_rcvdBuff[RFT_UART_RX_BUFF_SIZE];
	unsigned char m_txBuff[RFT_UART_TX_BUFF_SIZE]; 
	
  // INTERNAL USE ONLY
	BOOL createComPort(char *devName, BYTE bPort);
	BOOL setupConnection(DWORD dwBaudrate, DWORD bByteSize );
	BOOL createDataEvent(void);

private:

	pthread_t   m_readThread;
	int fd;                         // Serial Port Handle in Linux
	static void commReadThread( CRT_RFT_UART *pThis );

};

#endif//_RT_RS232_H_
//////////////////////////////////////////////////////////////////////////////
// END OF FILE
