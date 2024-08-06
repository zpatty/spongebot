#include "RFT_IF_PACKET_Rev1.2.h"

#include <string.h>

CRT_RFT_IF_PACKET::CRT_RFT_IF_PACKET()
{
	memset(m_rcvd_product_name,     0x00, sizeof(m_rcvd_product_name));
	memset(m_rcvd_serial_number,    0x00, sizeof(m_rcvd_serial_number));
	memset(m_rcvd_firmware_version, 0x00, sizeof(m_rcvd_firmware_version));

	m_rcvd_curr_RX_ID = 0;	// current sensor's RX ID, Only CAN version
	m_rcvd_curr_TX_ID_1 = 0;  // current sensor's TX ID #1, Only CAN version
	m_rcvd_curr_TX_ID_2 = 0;  // current sensor's TX ID #2, Only CAN version
	m_rcvd_set_RX_ID = 0;		// setting value of sensor's RX ID, Only CAN version
	m_rcvd_set_TX_ID_1 = 0;   // setting value of sensor's TX ID #1, Only CAN version
	m_rcvd_set_TX_ID_2 = 0;	// setting value of sensor's TX ID #2, Only CAN version

	m_rcvd_curr_comm_baudrate = 0; // current baudrate
	m_rcvd_set_comm_baudrate = 0;  // setting value of baudrate

	m_rcvd_filter_type = 0;
	m_rcvd_filter_setting_value = 0;

	m_rcvd_tx_frq = 0;

	m_response_cmd = 0;
	m_response_result = 0;
	m_response_errcode = 0;

	memset(m_rcvdForce, 0x00, sizeof(m_rcvdForce));

	m_rcvdForceStatus = 0;

	memset(m_rcvdOverloadCnt, 0x00, sizeof(m_rcvdOverloadCnt));

	m_fDividerForce = FORCE_DIVIDER;
	m_fDividerTorque = TORQUE_DIVIDER;
}

CRT_RFT_IF_PACKET::~CRT_RFT_IF_PACKET()
{

}

void CRT_RFT_IF_PACKET::setDivider(float force, float torque)
{
	m_fDividerForce = force;
	m_fDividerTorque = torque;
}

bool CRT_RFT_IF_PACKET::DFG_read_product_name(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_PRODUCT_NAME;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_serial_number(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_SERIAL_NUMBER;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_firmware_version(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_FIRMWARE_VER;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_message_ID(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_ID;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_comm_baudrate(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_COMM_BAUDRATE;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_filter_type(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_FT_FILTER;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_force_once(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_FT_ONCE;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_force(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_FT_CONT;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_read_output_frq(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_CONT_OUT_FRQ;

	return true;
}



bool CRT_RFT_IF_PACKET::DFG_read_overload_count(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_GET_OVERLOAD_COUNT;

	return true;
}












bool CRT_RFT_IF_PACKET::DFG_set_message_ID(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char rxID, unsigned char txID_1, unsigned char txID_2)
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_SET_ID;
	data_field[1] = rxID;
	data_field[2] = txID_1;
	data_field[3] = txID_2;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_set_comm_baudrate(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char baud_type)
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_SET_COMM_BAUDRATE;
	data_field[1] = baud_type;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_set_filter_type(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char filter_type, unsigned char set_value)
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_SET_FT_FILTER;
	data_field[1] = filter_type;
	data_field[2] = set_value;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_set_stop_force_out(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE])
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_FT_CONT_STOP;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_set_output_frq(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char frq_type)
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_SET_CONT_OUT_FRQ;
	data_field[1] = frq_type;

	return true;
}

bool CRT_RFT_IF_PACKET::DFG_set_bias(unsigned char data_field[COMMAND_PACKET_DATA_FIELD_SIZE], unsigned char is_on)
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	data_field[0] = CMD_SET_BIAS;
	data_field[1] = is_on;

	return true;
}


void CRT_RFT_IF_PACKET::convertPacket_To_Force(unsigned char *rcvdPacket, float *force, float forceDivider, float torqueDivider)
{
	short raw;
	unsigned short temp;

	for (int idx = 0; idx < RFT_NUM_OF_FORCE; idx++)
	{
		temp = rcvdPacket[2 * idx] * 256 + rcvdPacket[2 * idx + 1];
		raw = (signed short)temp;

		if (idx < 3) // force
		{
			force[idx] = ((float)raw) / forceDivider;
		}
		else // torque
		{
			force[idx] = ((float)raw) / torqueDivider;
		}
	}

	// force status - overload.....
	m_rcvdForceStatus = rcvdPacket[12];
}


bool CRT_RFT_IF_PACKET::rcvd_data_field_processing(unsigned char data_field[RESPONSE_PACKET_DATA_FIELD_SIZE], unsigned char check_command_type)
{
	if (data_field == 0) // pointer validataion checking.
		return false;

	unsigned char rcvd_cmd_type = data_field[0]; // command type

	if (check_command_type != 0)
	{
		if (rcvd_cmd_type != check_command_type)
			return false;
	}

	bool prcs_result = true;

	switch (rcvd_cmd_type)
	{
	case CMD_GET_PRODUCT_NAME:
		for (int idx = 0; idx < RFT_PRODUCT_NAME_LENGTH; idx++)
			m_rcvd_product_name[idx] = data_field[idx + 1];
		break;
	case CMD_GET_SERIAL_NUMBER:
		for (int idx = 0; idx < RFT_SERIAL_NUMBER_LENGTH; idx++)
			m_rcvd_serial_number[idx] = data_field[idx + 1];
		break;
	case CMD_GET_FIRMWARE_VER:
		for (int idx = 0; idx < RFT_FIRMWARE_VER_LENGTH; idx++)
			m_rcvd_firmware_version[idx] = data_field[idx + 1];
		break;
	case CMD_SET_ID:
		m_response_cmd = rcvd_cmd_type;
		m_response_result = data_field[1];
		m_response_errcode = data_field[2];
		break;
	case CMD_GET_ID:
		m_rcvd_curr_RX_ID	= data_field[1];	// current sensor's RX ID, Only CAN version
		m_rcvd_curr_TX_ID_1 = data_field[2];	// current sensor's TX ID #1, Only CAN version
		m_rcvd_curr_TX_ID_2 = data_field[3];	// current sensor's TX ID #2, Only CAN version
		m_rcvd_set_RX_ID	= data_field[4];	// setting value of sensor's RX ID, Only CAN version
		m_rcvd_set_TX_ID_1	= data_field[5];	// setting value of sensor's TX ID #1, Only CAN version
		m_rcvd_set_TX_ID_2	= data_field[6];	// setting value of sensor's TX ID #2, Only CAN version
		break;
	case CMD_SET_COMM_BAUDRATE:
		m_response_cmd = rcvd_cmd_type;
		m_response_result = data_field[1];
		m_response_errcode = data_field[2];
		break;
	case CMD_GET_COMM_BAUDRATE:
		m_rcvd_curr_comm_baudrate = data_field[1]; // current baudrate
		m_rcvd_set_comm_baudrate  = data_field[2]; // setting value of baudrate
		break;
	case CMD_SET_FT_FILTER:
		m_response_cmd = rcvd_cmd_type;
		m_response_result = data_field[1];
		m_response_errcode = data_field[2];
		break;
	case CMD_GET_FT_FILTER:
		m_rcvd_filter_type			= data_field[1];
		m_rcvd_filter_setting_value = data_field[2];
		break;
	case CMD_FT_ONCE:
		m_response_cmd = rcvd_cmd_type;
		//convertPacket_To_Force((&data_field[1]), m_rcvdForce, FORCE_DIVIDER, TORQUE_DIVIDER);
		convertPacket_To_Force((&data_field[1]), m_rcvdForce, m_fDividerForce, m_fDividerTorque);
		break;
	case CMD_FT_CONT:
		m_response_cmd = rcvd_cmd_type;
		//convertPacket_To_Force((&data_field[1]), m_rcvdForce, FORCE_DIVIDER, TORQUE_DIVIDER);
		convertPacket_To_Force((&data_field[1]), m_rcvdForce, m_fDividerForce, m_fDividerTorque);
		break;
	case CMD_FT_CONT_STOP:
		// NO - RESPONSE PACKET
		break;
	case CMD_RESERVED_1:
		break;
	case CMD_RESERVED_2:
		break;
	case CMD_SET_CONT_OUT_FRQ:
		m_response_cmd = rcvd_cmd_type;
		m_response_result = data_field[1];
		m_response_errcode = data_field[2];
		break;
	case CMD_GET_CONT_OUT_FRQ:
		m_rcvd_tx_frq = data_field[1];
		break;
	case CMD_SET_BIAS:
		break;
	case CMD_GET_OVERLOAD_COUNT:
		m_response_cmd = rcvd_cmd_type;
		for (int i = 0; i < RFT_NUM_OF_FORCE; i++)
		{
			m_rcvdOverloadCnt[i] = data_field[i + 1];
		}
		break;
	default:
		prcs_result = false;
		break;
	}

	return prcs_result;
}


///////////////////////////////////////////////////////////////////////////////////////////
// functions for UART communication, checksum calculation, UART Packet generation
unsigned char CRT_RFT_IF_PACKET::calcChecksum(unsigned char *pkt_buff, int pkt_size)
{
	unsigned char checksum = 0;

	for (int idx = 1; idx < (pkt_size - 2); idx++) // except SOP, CHECKSUM and EOP
		checksum += pkt_buff[idx];

	return checksum;
}

// UPG(Uart Packet Generation) functions

bool CRT_RFT_IF_PACKET::UPG_read_product_name(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_product_name(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_serial_name(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_serial_number(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_firmware_version(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_firmware_version(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_comm_baudrate(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_comm_baudrate(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_filter_type(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_filter_type(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_force_once(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_force_once(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_force(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_force(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_output_frq(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_output_frq(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_read_overload_count(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_read_overload_count(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}


bool CRT_RFT_IF_PACKET::UPG_set_comm_baudrate(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char buad_type)
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_set_comm_baudrate(packet_buff + 1, buad_type);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_set_filter_type(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char filter_type, unsigned char set_value)
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_set_filter_type(packet_buff + 1, filter_type, set_value);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_set_stop_force_out(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE])
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_set_stop_force_out(packet_buff + 1);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_set_output_frq(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char frq_type)
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_set_output_frq(packet_buff + 1, frq_type);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UPG_set_bias(unsigned char packet_buff[UART_COMMAND_PACKET_SIZE], unsigned char is_on)
{
	if (packet_buff == 0)
		return false;

	// buffer clear
	for (int idx = 0; idx < UART_COMMAND_PACKET_SIZE; idx++)
		packet_buff[idx] = 0x00;

	packet_buff[0] = SOP;
	DFG_set_bias(packet_buff + 1, is_on);
	packet_buff[UART_COMMAND_PACKET_SIZE - 2] = calcChecksum(packet_buff, UART_COMMAND_PACKET_SIZE);
	packet_buff[UART_COMMAND_PACKET_SIZE - 1] = EOP;

	return true;
}

bool CRT_RFT_IF_PACKET::UART_packet_processing(unsigned char packet_buff[UART_RESPONSE_PACKET_SIZE], unsigned char check_command_type)
{
	if (packet_buff == 0)
		return false;

	unsigned char checksum = calcChecksum(packet_buff, UART_RESPONSE_PACKET_SIZE);
	if ((packet_buff[0] == SOP)
		&& (packet_buff[UART_RESPONSE_PACKET_SIZE - 2] == checksum)
		&& (packet_buff[UART_RESPONSE_PACKET_SIZE - 1] == EOP))
	{
		if (rcvd_data_field_processing(packet_buff + 1, check_command_type))
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}

	return true;
}

// END OF FILE
