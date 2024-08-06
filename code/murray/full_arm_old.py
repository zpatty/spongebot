#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import math
from math import cos, sin
from datetime import datetime
import numpy as np
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from Dynamixel import *                        # Dynamixel motor class                                  
from dyn_functions import *                    # Dynamixel support functions
from puppet_controller import *                # Controller 
from logger import *                           # Data Logger
from Constants import *                        # File of constant variables

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
    def kbhit():
        return msvcrt.kbhit()
else:
    import termios, fcntl, sys, os
    from select import select
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    new_term = termios.tcgetattr(fd)

    def getch():
        new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        return ch

    def kbhit():
        new_term[3] = (new_term[3] & ~(termios.ICANON | termios.ECHO))
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            dr,dw,de = select([sys.stdin], [], [], 0)
            if dr != []:
                return 1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            sys.stdout.flush()

        return 0

# Open module port
if portHandlerMod.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandlerMod.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# open big motors port
# Open joint port
if portHandlerJoint.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandlerJoint.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

M1 = Dynamixel(packetHandlerMod, portHandlerMod, 1)
M1.current_control_mode()
M1.enable_torque()
M2 = Dynamixel(packetHandlerMod, portHandlerMod, 2)
M2.current_control_mode()
M2.enable_torque()
M3 = Dynamixel(packetHandlerMod, portHandlerMod, 3)
M3.current_control_mode()
M3.enable_torque()
Mod1 = [M1, M2, M3]
print("MOD 1 SETUP SUCCESS")

M4 = Dynamixel(packetHandlerMod, portHandlerMod, 4)
M4.current_control_mode()
M4.enable_torque()
M5 = Dynamixel(packetHandlerMod, portHandlerMod, 5)
M5.current_control_mode()
M5.enable_torque()
M6 = Dynamixel(packetHandlerMod, portHandlerMod, 6)
M6.current_control_mode()
M6.enable_torque()
Mod2 = [M4, M5, M6]
print("MOD 2 SETUP SUCCESS")

M7 = Dynamixel(packetHandlerMod, portHandlerMod, 7)
M7.current_control_mode()
M7.enable_torque()
M8 = Dynamixel(packetHandlerMod, portHandlerMod, 8)
M8.current_control_mode()
M8.enable_torque()
M9 = Dynamixel(packetHandlerMod, portHandlerMod, 9)
M9.current_control_mode()
M9.enable_torque()
Mod3 = [M7, M8, M9]
print("MOD 3 SETUP SUCCESS")

# initialize base motor
BaseMotor = Dynamixel(packetHandlerJoint, portHandlerJoint, 0)
BaseMotor.current_control_mode()
BaseMotor.enable_torque()
print("BASE MOTOcurrent_control_mode()R SETUP SUCCESS")
# initialize joints
Joint1 = Dynamixel(packetHandlerJoint, portHandlerJoint, 1)
Joint1.current_control_mode()
Joint1.enable_torque()
print("JOINT 1 MOTOR SETUP SUCCESS")

Joint2 = Dynamixel(packetHandlerJoint, portHandlerJoint, 2)
Joint2.current_control_mode()
Joint2.enable_torque()
print("JOINT2 MOTOR SETUP SUCCESS")

# MAKE SURE TO UNCOMMENT THIS WHEN YOURE DONE
tc = get_time()
test = Mod1[0].get_present_pos()
tcc = get_time()
dcc = (tcc - tc).total_seconds()   
print(f"dccccc: {dcc}\n")   
th1_0 = [to_radians(Mod1[0].get_present_pos()), to_radians(Mod1[1].get_present_pos()), to_radians(Mod1[2].get_present_pos())]
th2_0 = [to_radians(Mod2[0].get_present_pos()), to_radians(Mod2[1].get_present_pos()), to_radians(Mod2[2].get_present_pos())]
th3_0 = [to_radians(Mod3[0].get_present_pos()), to_radians(Mod3[1].get_present_pos()), to_radians(Mod3[2].get_present_pos())]
# offset1 = to_radians(Joint1.get_present_pos())
offset1 = 3.710699525895366
# offset2 = to_radians(Joint2.get_present_pos())
offset2 = 4.368777283898306

BigMotors = [to_radians(BaseMotor.get_present_pos()), to_radians(Joint1.get_present_pos()) - offset1, to_radians(Joint2.get_present_pos()) - offset2]

print(f"Mod 1 Reference Angles: {th1_0}\n")
print(f"Mod 2 Reference Angles: {th2_0}\n")
print(f"Mod 3 Reference Angles: {th3_0}\n")

print(f"Z Motor Position: {BigMotors[0]}")
print(f"Offset for Motor Positions 1 and 2: {offset1, offset2}")

# our max arc length (in m)
# s = (l3_0[0] + l3_0[1] + l3_0[2])/3
s = 0.08
# L1 = s/5 
# L3 = s/5 
# L5 = s/5 
# L6 = s/5 
# L8 = s/5 

# initialize variables 
l = [[0,0,0], [0,0,0], [0,0,0]]
# cables of the three mods at neutral position 
l0 = [l1_0, l2_0, l3_0]
# the motor angles of the 9 motors at neutral position 
th0 = [th1_0, th2_0, th3_0]
# grab cable lengths of all 9 motors
Mods = [Mod1, Mod2, Mod3]
l = grab_arm_cable_lens(Mods, l, l0, th0, r)
print(f"Cable lengths based off of dtheta: {l}\n")
print(f" big motors angles: {BigMotors}")
q = grab_arm_q(l[0], l[1], l[2], BigMotors[0], BigMotors[1], BigMotors[2], s, d)

q_old = q

# Report our initial configuration
print(f"Our current q: {q}\n")
first_time = True

# Start the control loop
try: 
    while 1:
        print("\nPress S to set your qd or W to run your trajectory!(or press SPACE to quit!)")
        key_input = getch()
        if key_input == chr(SPACE_ASCII_VALUE):
            print("we're quitting\n")
            break
        elif key_input == chr(SKEY_ASCII_VALUE):
            
            dx = float(input("Desired dx: "))
            dy = float(input("Desired dy: "))
            dL = float(input("Desired dL: "))
            qd = np.array([dx, dy, dL]).reshape(-1,1)
            zero =  np.array([0, 0, 0]).reshape(-1,1)
            t_old = get_time()
            # our loop's "starting" time
            t_0 = get_time()
            while 1:
                if kbhit():
                    c = getch()
                    if c == chr(QKEY_ASCII_VALUE):
                        first_time = True
                    for i in range(len(Mod1)):
                        Mod1[i].send_torque_cmd(0) 
                    l1 = grab_cable_lens(Mod1, l1, l1_0, th1_0, r)
                    print(f"what cable lengths the mod reads: {l1}\n")
                    break
    
                else:
                    # grab current time
                    t = get_time()

                    # mod clock keeps time since loop has started
                    mod_clock = (t - t_0).total_seconds()
                    l1 = grab_cable_lens(Mod1, l1, l1_0, th1_0, r)
                    l11 = l1[0]
                    l12 = l1[1]
                    l13 = l1[2]
                    # l1 = [l13, l11, l12]
                    # log cable lengths and current configuration of module
                    q = grab_q(l1, s, d)
                    dt = (t - t_old).total_seconds()     
                    t_old = t  

                    # At the first iteration velocity is 0  
                    if first_time:
                        q_dot = np.array([0, 0, 0]).reshape(-1,1)
                        first_time = False
                    else:
                        q_dot = diff(q, q_old, dt)       
                    # calculate errors
                    err = q - qd
                    err_dot = q_dot
                    print(f"q: {q}\n")
                    # print(f"qd: {qd}\n")
                    # calculate and record torque

                    tau_cables = puppet_controller(q, q_dot, qd, zero, zero, d, hp, mplate, s, Kpb=15, Kps=1500, ks=2000)
                    # print(f"tau: {tau_cables.reshape(1,-1)}\n")
                    tau_cables = np.maximum(3 * tau_cables,np.array([[-30],[-30],[-30]]))

                    q_old = q

                    mod_input = grab_current(tau_cables, min_torque, max_torque)
                    # print(f"mod_input: {mod_input}\n")                
                    # send current command to motors
                    
                    Mod1[2].send_torque_cmd(mod_input[2])  
                    Mod1[1].send_torque_cmd(mod_input[1])  
                    Mod1[0].send_torque_cmd(mod_input[0])  
                        
        elif key_input == chr(WKEY_ASCII_VALUE):    # print out the length changes
            # k1 = float(input("Desired k1: "))
            # k2 = float(input("Desired k2: "))
            # k3 = float(input("Desired k3: "))
            # phi1 = float(input("Desired phi1: "))
            # phi2 = float(input("Desired phi2: "))
            # phi3 = float(input("Desired phi3: "))
            # dL1 = float(input("Desired dL1: "))
            # dL2 = float(input("Desired dL2: "))
            # dL3 = float(input("Desired dL3: ")) 
            # zmotor = float(input("Desired zmotor: "))
            # jm1 = float(input("Desired jm1: "))
            # jm2 = float(input("Desired jm2: "))
            # qd = grab_qd(k1,k2,k3,phi1, phi2, phi3, dL1, dL2, dL3, zmotor, jm1, jm2)
            qd = np.zeros((12,1)).reshape(-1,1)
            print(f"Our desired config: {qd}\n")
            print(f"Our error qd - q: {qd - q}")
            zero =  np.zeros((12,1))
            t_old = get_time()
            # our loop's "starting" time
            t_0 = get_time()
            while 1:
                if kbhit():
                    c = getch()
                    if c == chr(QKEY_ASCII_VALUE):
                        first_time = True
                    for i in range(len(Mods)):
                        Mods[i][0].send_torque_cmd(0)
                        Mods[i][1].send_torque_cmd(0)
                        Mods[i][2].send_torque_cmd(0) 
                    BaseMotor.send_torque_cmd(0)
                    Joint1.send_torque_cmd(0)
                    Joint2.send_torque_cmd(0)
                    print("[Q KEY PRESSED] : All motors stopped\n")
                    break
    
                else:
                    # grab current time
                    t = get_time()

                    # mod clock keeps time since loop has started
                    # mod_clock = (t - t_0).total_seconds()
                    # this takes 0.148766
                    t_told = get_time()
                    l = grab_arm_cable_lens(Mods, l, l0, th0, r)     
                    tt = get_time()
                    dtt = (tt - t_told).total_seconds()   
                    print(f"dtt: {dtt}\n") 
                    # 0.01 seconds
                    mj1 = to_radians(Joint1.get_present_pos()) - offset1
                    mj2 = to_radians(Joint2.get_present_pos()) - offset2
                    
                    # this is 0.020438
                    q = grab_arm_q(l[0], l[1], l[2], to_radians(BaseMotor.get_present_pos()), mj1, mj2, s, d)
                    
                    dt = (t - t_old).total_seconds()   
                    print(f"dt: {dt}\n")  
                    t_old = t  

                    # At the first iteration velocity is 0  
                    if first_time:
                        dq = np.zeros((12,1))
                        first_time = False
                    else:
                        dq = diff(q, q_old, dt)    
                    # calculate errors
                    err = q - qd
                    err_dot = dq
                    # print(f"q: {q}\n")
                    # calculate and record torque

                    # tau_cables = puppet_controller(q, q_dot, qd, zero, zero, d, hp, mplate, s, Kpb=15, Kps=1500, ks=2000)
                    mm = 0.12
                    hm = 0.06
                    rm = 0.02
                    N = 6
                    kb = 1
                    ks = 20
                    bb = 1
                    bs = 1
                    e = 1/1000
                    kp = 0
                    kl = 0
                    km = 15
                    Kp = np.diag([0,kp,kp,kl,112,kp,kp,kl,km,kp,kp,kl])*0
                    KD = 12
                    kc = 10000
                    # print([q, dq,qd,zero,zero,d,hp,mplate,mm,hm,rm,r,N,kb,ks,bb,bs,s,e,Kp,KD,kc])
                    # TODO: set position limits
                    
                    tau = puppet_controller(q,dq,qd,zero,zero,d,hp,mplate,mm,hm,rm,r,N,kb,ks,bb,bs,s,e,Kp,KD,kc)
                    
                    # print(f"tau: {tau.reshape(1,-1)}\n")
                    # tc = get_time()
                    # negligible
                    tau_clip = np.concatenate((np.zeros((1,1)), tau[1:4], np.zeros((1,1)), tau[5:8],np.zeros((1,1)), tau[9:]))
                    
                    tau_cables = np.maximum(3 * tau_clip*0,-30 * np.ones((12,1)))
                    tau_cables[0,0] = tau[0,0]
                    tau_cables[4,0] = tau[4,0]
                    tau_cables[8,0] = tau[8,0]
                    # print(f"tau cables: {tau_cables}\n")
                    q_old = q
                    # 0.001
                    arm_input = grab_arm_current(tau_cables, min_torque, max_torque)
                    
                    # print(f"arm_input: {arm_input}\n")                
                    # send current command to motors
                    base_cmd = arm_input[0]
                    mj1_cmd = arm_input[4]
                    mj2_cmd = arm_input[8]
                    mod1_cmd = arm_input[1:4]

                    # print(f"mod 1 cmd: {mod1_cmd}\n")
                    mod2_cmd = arm_input[5:8]
                    mod3_cmd = arm_input[9:]
                    # sending torque commands take 0.18
                    for i in range(len(Mod1)):
                        Mod1[i].send_torque_cmd(mod1_cmd[i])
                        # print(f"For motor {i + 1} sending {mod1_cmd[i]}\n")
                        Mod2[i].send_torque_cmd(mod2_cmd[i])
                        Mod3[i].send_torque_cmd(mod3_cmd[i])
                    tc = get_time()
                    # single torque command takes 0.006
                    BaseMotor.send_torque_cmd(base_cmd)
                    Joint1.send_torque_cmd(mj1_cmd)
                    Joint2.send_torque_cmd(mj2_cmd)

    print("Disabling torque")
    # Disable Dynamixel Torque
    M1.disable_torque()
    M2.disable_torque()
    M3.disable_torque()
    M4.disable_torque()
    M5.disable_torque()
    M6.disable_torque()
    M7.disable_torque()
    M8.disable_torque()
    M9.disable_torque()
    BaseMotor.disable_torque()
    Joint1.disable_torque()
    Joint2.disable_torque()

    # Close port
    # portHandlerMod.closePort()
except:
    print("ERROR: disabling torque")
    M1.disable_torque()
    M2.disable_torque()
    M3.disable_torque()
    M4.disable_torque()
    M5.disable_torque()
    M6.disable_torque()
    M7.disable_torque()
    M8.disable_torque()
    M9.disable_torque()
    BaseMotor.disable_torque()
    Joint1.disable_torque()
    Joint2.disable_torque()