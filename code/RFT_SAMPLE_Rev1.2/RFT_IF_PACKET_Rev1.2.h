/*
	Rev1.0, Release Version
	Rev1.1, 2017.07.27
	 - Add the divider setting function
	Rev1.2, 2017.12.14
	 - Modified type error of function name.
*/

#ifndef __RT_RFT_PACKET_H__
#define __RT_RFT_PACKET_H__

#define CMD_NONE					(0)

// command types
#define CMD_GET_PRODUCT_NAME		(1)
#define CMD_GET_SERIAL_NUMBER		(2)
#define CMD_GET_FIRMWARE_VER		(3)
#define CMD_SET_ID					(4)	// Only CAN version
#define CMD_GET_ID					(5) // Only CAN version
#define CMD_SET_COMM_BAUDRATE		(6) // Only UART version, CAN : 1Mbps Fixed
#define CMD_GET_COMM_BAUDRATE		(7) 
#define CMD_SET_FT_FILTER			(8)
#define CMD_GET_FT_FILTER			(9)
#define CMD_FT_ONCE					(10) 
#define CMD_FT_CONT					(11) 
#define CMD_FT_CONT_STOP			(12)
#define CMD_RESERVED_1				(13)
#define CMD_RESERVED_2				(14)
#define CMD_SET_CONT_OUT_FRQ 		(15)	
#define CMD_GET_CONT_OUT_FRQ  		(16)	
#define CMD_SET_BIAS				(17)
#define CMD_GET_OVERLOAD_COUNT		(18)

#define RFT_NUM_OF_FORCE			(6)

#define RFT_PRODUCT_NAME_LENGTH		(15) 
#define RFT_SERIAL_NUMBER_LENGTH	(15)
#define RFT_FIRMWARE_VER_LENGTH		(15)


#define RESPONSE_CAN_PACKET_CNT			(2)
#define COMMAND_PACKET_DATA_FIELD_SIZE	(8)
#define RESPONSE_PACKET_DATA_FIELD_SIZE (16)

// OUTPUT FRQ. DEFINITIONS
#define OUTPUT_FRQ_200Hz_DEFAULT	(0)
#define OUTPUT_FRQ_10Hz				(1)
#define OUTPUT_FRQ_20Hz				(2)
#define OUTPUT_FRQ_50Hz				(3)
#define OUTPUT_FRQ_100Hz			(4)
#define OUTPUT_FRQ_200Hz			(5)
#define OUTPUT_FRQ_300Hz			(6)
#define OUTPUT_FRQ_500Hz			(7)
#define OUTPUT_FRQ_1000Hz			(8)

// FILTER SET VALUE DEFINITIONS
#define FILTER_NONE			(0) // NONE
#define FILTER_1ST_ORDER_LP	(1) // 1st order low-pass filter

// Cutoff Frq.
#define CUTOFF_1Hz		(1)
#define CUTOFF_2Hz		(2)
#define CUTOFF_3Hz		(3)
#define CUTOFF_5Hz		(4)
#define CUTOFF_10Hz		(5)
#define CUTOFF_20Hz		(6)
#define CUTOFF_30Hz		(7)
#define CUTOFF_40Hz		(8)
#define CUTOFF_50Hz		(9)
#define CUTOFF_100Hz	(10)
#define CUTOFF_150Hz	(11)
#define CUTOFF_200Hz	(12)
#define CUTOFF_300Hz	(13)
#define CUTOFF_500Hz	(14)

// BAUDRATE DEFINITIONS
#define CAN_BAUDRATE_1MBPS (0) // 1Mbps, CAN FIXED BIT-RATE
// BAUDRATE FOR UART VERSIONS
#define UART_BAUDRATE_115200_DEFAULT (0)
#define UART_BAUDRATE_921600 (1)
#define UART_BAUDRATE_460800 (2)
#define UART_BAUDRATE_230400 (3)
#define UART_BAUDRATE_115200 (4)
#define UART_BAUDRATE_57600  (5)

// ERROR CODE.
#define ERROR_NONE				(0)
#define NOT_SUPPORTED_CMD		(1)
#define SET_VALUE_RANGE_ERR		(2)
#define EEPROM_WRITING_ERR		(3)


//
#define FORCE_DIVIDER (50.0f)		// default value
#define TORQUE_DIVIDER (2000.0f)	// default value

//
#define OVERLOAD_BIT_FX (0x20)
#define OVERLOAD_BIT_FY (0x10)
#define OVERLOAD_BIT_FZ (0x08)
#define OVERLOAD_BIT_TX (0x04)
#define OVERLOAD_BIT_TY (0x02)
#define OVERLOAD_BIT_TZ (0x01)


//////////////////////////////////////////////////
// FOR UART PACKET
#define SOP (0x55)
#define EOP (0xAA)

#define UART_COMMAND_PACKET_SIZE	(11) // SOP(1) + DATA FIELD(8)  + CHECK_SUM(1) + EOP(1)
#define UART_RESPONSE_PACKET_SIZE	(19) // SOP(1) + DATA FIELD(16) + CHECK_SUM(1) + EOP(1)

class CRT_RFT_IF_PACKET
{
public:
	// constructor and destructor
	CRT_RFT_IF_PACKET();
	~CRT_RFT_IF_PACKET();

public:
	float m_fDividerForce;
	float m_fDividerTorque;
	char m_rcvd_product_name[RFT_PRODUCT_NAME_LENGTH + 1];	// ASCII CODE VALUE
	char m_rcvd_serial_number[RFT_SERIAL_NUMBER_LENGTH + 1];	// ASCII CODE VALUE
	char m_rcvd_firmware_version[RFT_FIRMWARE_VER_LENGTH + 1];// ASCII CODE VALUE

	unsigned char m_rcvd_curr_RX_ID;	// current sensor's RX ID, Only CAN version
	unsigned char m_rcvd_curr_TX_ID_1;  // current sensor's TX ID #1, Only CAN version
	unsigned char m_rcvd_curr_TX_ID_2;  // current sensor's TX ID #2, Only CAN version
	unsigned char m_rcvd_set_RX_ID;		// setting value of sensor's RX ID, Only CAN version
	unsigned char m_rcvd_set_TX_ID_1;   // setting value of sensor's TX ID #1, Only CAN version
	unsigned char m_rcvd_set_TX_ID_2;	// setting value of sensor's TX ID #2, Only CAN version

	unsigned char m_rcvd_curr_comm_baudrate; // current baudrate
	unsigned char m_rcvd_set_comm_baudrate;	 // setting value of baudrate

	unsigned char m_rcvd_filter_type;			// received filter type
	unsigned char m_rcvd_filter_setting_value;	// received filter setting value

	unsigned char m_rcvd_tx_frq;				// received transmit frq.

	unsigned char m_response_cmd;				// received command id of response packet
	unsigned char m_response_result;			// received response result of response packet
	unsigned char m_response_errcode;			// received error code of respose packet

	float m_rcvdForce[RFT_NUM_OF_FORCE];		// received F/T values
	unsigned short m_rcvdForceStatus;			// received F/T overload status 

	unsigned char m_rcvdOverloadCnt[RFT_NUM_OF_FORCE]; // received F/T overload count

public:

	void setDivider(float force, float torque);

	

	//////////////////////////////////////////////////////////////////////////////////////////
	// Data Field Generation(DFG) of command packet

	bool DFG_read_product_name(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_serial_number(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_firmware_version(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_message_ID(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_comm_baudrate(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_filter_type(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_force_once(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_force(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_output_frq(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_read_overload_count(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);

	bool DFG_set_message_ID(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char rxID, unsigned char txID_1, unsigned char txID_2);
	bool DFG_set_comm_baudrate(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char baud_type );
	bool DFG_set_filter_type(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char filter_type, unsigned char set_value );
	bool DFG_set_stop_force_out(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE]);
	bool DFG_set_output_frq(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char frq_type);
	bool DFG_set_bias(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char is_on);

	///////////////////////////////////////////////////////////////////////////////////////////
	// data field processing

	bool rcvd_data_field_processing(unsigned char data_field[RESPONSE_PACKET_DATA_FIELD_SIZE], unsigned char check_command_type = 0);

	///////////////////////////////////////////////////////////////////////////////////////////
	// functions for UART communication, checksum calculation, UART Packet generation
	// UPG(Uart Packet Generation) functions

	bool UPG_read_product_name(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_serial_name(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_firmware_version(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_comm_baudrate(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_filter_type(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_force_once(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_force(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_output_frq(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_read_overload_count(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);

	bool UPG_set_comm_baudrate(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char buad_type);
	bool UPG_set_filter_type(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char filter_type, unsigned char set_value);
	bool UPG_set_stop_force_out(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE]);
	bool UPG_set_output_frq(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char frq_type);
	bool UPG_set_bias(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char is_on);

	bool UART_packet_processing(unsigned char packet_buff[UART_RESPONSE_PACKET_SIZE], unsigned char check_command_type = 0);

protected:
	// received packet data processing functions

	void convertPacket_To_Force(unsigned char *rcvdPacket, float *force, float forceDivider, float torqueDivider);

	// check sum calculation for UART PACKET
	unsigned char calcChecksum(unsigned char *pkt_buff, int pkt_size);

};

#endif//__RT_RFT_PACKET_H__

// END OF FILE
