#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from Dynamixel import *                        # Dynamixel motor class
import math
# KEYBOARD INPUTS
ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20
WKEY_ASCII_VALUE            = 0x77
OKEY_ASCII_VALUE            = 0x6F
SKEY_ASCII_VALUE            = 0x73
AKEY_ASCII_VALUE            = 0x61
DKEY_ASCII_VALUE            = 0x64
CKEY_ASCII_VALUE            = 0x63
BKEY_ASCII_VALUE            = 0x62      # key to bend the top module
UKEY_ASCII_VALUE            = 0x75      # key to unbend the modules
NKEY_ASCII_VALUE            = 0x6E
IKEY_ASCII_VALUE            = 0x69     
QKEY_ASCII_VALUE            = 0x71 
TKEY_ASCII_VALUE            = 0x74     
MOD1_VALUE                  = 0x31      # pressing 1 on keyboard
MOD2_VALUE                  = 0x32
MOD3_VALUE                  = 0x33

# ID that commands all dynamixels
DXL_IDALL                   = 254
# Protocol version
PROTOCOL_VERSION            = 2.0            # See which protocol version is used in the Dynamixel
MOD_DEVICE                  = '/dev/ttyUSB2'
JOINTS                      = '/dev/ttyUSB0'
BAUDRATE                    = 2000000
portHandlerMod              = PortHandler(MOD_DEVICE)
packetHandlerMod            = PacketHandler(PROTOCOL_VERSION)
portHandlerJoint            = PortHandler(JOINTS)
packetHandlerJoint          = PacketHandler(PROTOCOL_VERSION)
MAX_VELOCITY = 20

# some constants 
d = 0.04337                    # circumradius of hexagon plate (m)
r = 0.004                      # spool radius (m)
err = math.inf                 # start with large error
mass_module = .05              # in grams
mj = mass_module*4
m = mj
mplate = 0.08          # in kilograms
hp = 0.007
Lm = 0.013             # distance between center of spool and centroid of bottom plate
# lengths when modules are not stressed (in m)
L0 = 0.09
l1_0 = [L0, L0, L0]
l2_0 = [L0, L0, L0]
l3_0 = [L0, L0, L0]
l4_0 = [L0, L0, L0]


limits = [l1_0, l2_0, l3_0, l4_0]
# our reference motor angles (when arm is neutral, a.k.a standing)
th1_0 = [-0.24844533221769077, 0.0883800332630868, 0.1038916469901864] #[9.618059540042971, 5.717146396449785, 3.373223752560525]
th2_0 = [0.11797972654612812, 0.12016411518807728, 0.1396886226562857] #[6.968874719364468, 6.040816342693655, 4.201573378018771]
th3_0 = [0.08796279048878189, 0.10599626863116549, 0.09355261647983718] #[2.4911847995262812, 2.8363304768005504, 6.011670707723828]
th4_0 = [0.11496085235556917, 0.11867922178540397, 0.11385025026513998] #[5.375068680751287, 6.712699927787566, 3.2949907323783574]
calibration = [[2.81178678419438, 2.9789906900739154, 1.1658253987930873], [0.36355344672889695, 1.000155473701438, 0.23930100291016002], [3.9131849898962705, 1.7717478100079156, 2.8486023231036355], [1.2087768608538851, 0.5691068723055729, 5.0958841773561]]
# min and max are in mA because Dynamixel takes in mA inputs for current control
max_torque = 250
xm_max_torque = 1100
min_torque = 5
xm_min_torque = 2