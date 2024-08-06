import sys
import time
sys.path.append("build/")
from rft import CRT_RFT_UART
# from rft import CRT_RFT_IF_PACKET

devName = "/dev/ttyUSB"
port = 0
baud = 115200
byte_size = 8

g_force_divider 	= 50.0
g_torque_divider 	= 2000.0

RFT_SENSOR = CRT_RFT_UART()
# RFT_SENSOR.m_RFT_IF_PACKET = CRT_RFT_IF_PACKET()
RFT_SENSOR.openPort( devName, port, baud )
# RFT_SENSOR.CRT_RFT_IF_PACKET.setDivider(g_force_divider, g_torque_divider)

time.sleep(1)

RFT_SENSOR.rqst_FT_Continuous()



for i in range(10):
    time.sleep(0.25)
    # worked = RFT_SENSOR.rcvd_FT()
    # print(worked)
    print(f"Fx, Fy, Fz, Tx, Ty, Tz: {RFT_SENSOR.force_out}\n")

RFT_SENSOR.rqst_FT_Stop()

RFT_SENSOR.closePort()

