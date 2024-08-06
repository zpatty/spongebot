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
from puppet_controller_one_mod import *                # Controller 
from murray_controller import *                # Controller from MLS
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

M1 = Dynamixel(packetHandlerMod, portHandlerMod, 7)
M1.current_control_mode()
M1.enable_torque()
M2 = Dynamixel(packetHandlerMod, portHandlerMod, 8)
M2.current_control_mode()
M2.enable_torque()
M3 = Dynamixel(packetHandlerMod, portHandlerMod, 9)
M3.current_control_mode()
M3.enable_torque()
# Mod1 = [M3, M1, M2]
Mod1 = [M1, M2, M3]

# MAKE SURE TO UNCOMMENT THIS WHEN YOURE DONE
# th1_0 = [to_radians(Mod1[0].get_present_pos()), to_radians(Mod1[1].get_present_pos()), to_radians(Mod1[2].get_present_pos())]

print(f"Initial Reference Angles: {th1_0}\n")
# our max arc length (in m)
s = (l1_0[0] + l1_0[1] + l1_0[2])/3
L1 = s/5 
L3 = s/5 
L5 = s/5 
L6 = s/5 
L8 = s/5 

# initialize variables 
l1 = [0, 0, 0]
l1 = grab_cable_lens(Mod1, l1, l1_0, th1_0, r)
print(f"Cable lengths based off of dtheta: {l1}\n")

q = grab_q_murray(l1, s)
q_old = q

# Report our initial configuration
print(f"Our current q: {q}\n")
first_time = True

# Start the control loop
while 1:
    print("\nPress S to set your qd or W to run your trajectory!(or press SPACE to quit!)")
    key_input = getch()
    if key_input == chr(SPACE_ASCII_VALUE):
        print("we're quitting\n")
        break
    elif key_input == chr(SKEY_ASCII_VALUE):
        # ZACH CONTROLLER
        dx = float(input("Desired dx: "))
        dy = float(input("Desired dy: "))
        dL = float(input("Desired dL: "))
        # Kpb= float(input("Kpb: "))
        # Kps= float(input("Kps: ")) 
        # ks=  float(input("ks: "))
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
        # EMILY CONTROLLER
        cable1 = float(input("Cable Length 1: "))
        cable2 = float(input("Cable Length 2: "))
        cable3 = float(input("Cable Length 3: "))
        qd = grab_q_murray([cable1, cable2, cable3],s)
        print(f"Our qd: {qd}\n")
        zero =  np.zeros((9,1))
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
                q = grab_q_murray(l1, s)
                print(f"q: {q}\n")
                dt = (t - t_old).total_seconds()     
                t_old = t  
                # At the first iteration velocity is 0  
                if first_time:
                    q_dot = zero
                    first_time = False
                else:
                    q_dot = diff(q, q_old, dt)       
                tau_cables = 3000 * murray_controller(q, q_dot, qd, zero, zero, d, hp, m, s, kp=50, kv=75, ks=50)
                print(f"tau: {tau_cables.reshape(1,-1)}\n")
                Km = 30
                # .003 * 
                tau_cables = np.maximum(tau_cables,np.array([[-30],[-30],[-30]]))
                print(f"tau_cables: {tau_cables}\n")
                q_old = q

                mod_input = grab_current(tau_cables, min_torque, max_torque)
                print(f"mod_input: {mod_input}\n")                
                # send current command to motors
                
                Mod1[2].send_torque_cmd(mod_input[2])  
                Mod1[1].send_torque_cmd(mod_input[1])  
                Mod1[0].send_torque_cmd(mod_input[0])  
print("Disabling torque")
# Disable Dynamixel Torque
M1.disable_torque()
M2.disable_torque()
M3.disable_torque()

# Close port
# portHandlerMod.closePort()