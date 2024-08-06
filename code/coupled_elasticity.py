#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import math
from math import cos, sin
from datetime import datetime
import numpy as np
from matplotlib import pyplot as plt
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from Dynamixel import *                        # Dynamixel motor class                                  
from dyn_functions import *                    # Dynamixel support functions
from turtle_controller import *                # Controller 
# from logger import *                           # Data Logger
from Constants import *                        # File of constant variables
from Mod import *
from utilities import *
import json
import traceback
from queue import Queue
import serial
import socket
from scipy.ndimage import uniform_filter1d
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

# os.system('sudo /home/zach/git-repos/Documents/drl-turtle/motors/latency_write.sh')


host = '' #Server ip
port = 4000

# echo-server.py




    
    

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

# packetHandlerMod = PacketHandler(PROTOCOL_VERSION)
packetHandlerJoint = PacketHandler(PROTOCOL_VERSION)

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

IDs = [1]
Joints = Mod(packetHandlerJoint, portHandlerJoint, IDs)
Joints.set_current_cntrl_mode()
Joints.enable_torque()
t_old = time.time()

print("[DEBUG] dt:", (time.time() - t_old))  

nq = len(IDs)
th0 = 3.14/180 * np.array([180.0])#[180, 180, 270, 180, 180, 0]
# offset1 = joint_th0[1]
# offset1 = 3.745981084016736
# offset2 = joint_th0[2]
# offset2 = 4.414796707534875
# print(f"Offset for Motor Positions 1 and 2: {offset1, offset2}")
q = th0
q_old = q
# print(f"Motor Positions: {q}")
# our max arc length (in m)

print(f"Motor angles: {q}")
q_data = np.zeros((nq,1))
q2_data = np.zeros((nq,1))
tau_data = np.zeros((nq,1))
timestamps = np.zeros((1,1))
dt_loop = np.zeros((1,1))       # hold dt data 

# Report our initial configuration
q = np.array(Joints.get_position()).reshape(-1,1)
print(f"Our current q: {q}\n")
first_time = True
input_history = np.zeros((nq,10))


    # with conn:
    #     print(f"Connected by {addr}")
            
            # conn.sendall(data)
ft_sensor = False
try: 
    if ft_sensor:
        HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
        PORT = 10000  # Port to listen on (non-privileged ports are > 1023)

        s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
    while 1:
        print("\nP: Position Controlled Trajectory, C: Current Controlled Trajectory, W: Set point(or press SPACE to quit!)")
        key_input = getch()
        if key_input == ' ':
            print("we're quitting\n")
            break
        elif key_input == 'p':
            Joints.disable_torque()
            Joints.set_extended_pos_mode()
            Joints.enable_torque()
            # print("\n1: Walk, 2: Swim (or press SPACE to quit!)")
            # c
            if key_input == chr(SPACE_ASCII_VALUE):
                print("we're quitting\n")
                break
            qd_mat = mat2np('qd_p.mat', 'qd')
            tvec = mat2np('tvec_p.mat', 'tvec')
            t_0 = time.time()
            while 1:
                key_input = getch()
                if key_input == 'q':
                    c = getch()
                    first_time = True
                    Joints.send_torque_cmd(nq * [0])
                    print("[Q KEY PRESSED] : All motors stopped\n")
                    # save_data(q_data, qd, tau_data, timestamps, config_params, dt_loop)

                    break
                else:
                    n = get_qindex((time.time() - t_0), tvec)
                    
                    qd = np.array(qd_mat[:, n]).reshape(-1,1)
                    Joints.send_pos_cmd(np.squeeze(to_motor_steps(qd)))
                    
            
        elif key_input == 'b':
            # Back to home position 
            # put all motors in extended position mode
            # put them back into their reference motor step position
            Joints.disable_torque()
            Joints.set_extended_pos_mode()
            Joints.enable_torque()
            Joints.set_max_velocity(20)
            home = []
            for i in range(len(th0)):
                for j in range(len(th0[0])):
                    home.append(to_motor_steps(th0[i][j]))
            Joints.send_pos_cmd(home)
            break
        elif key_input == chr(WKEY_ASCII_VALUE):    # print out the length changes
            Joints.disable_torque()
            Joints.set_current_cntrl_mode()
            Joints.enable_torque()
            q_data = np.zeros((nq,1))
            tau_data = np.zeros((nq,1))
            F_model_data = np.zeros((nq,10))
            F_fb_data = np.zeros((nq,10))
            timestamps = np.zeros((1,1))
            # Open up controller parameters
            # Kp, KD, config_params = parse_config()
            Kp = np.diag([0.1])
            KD = 0.07
            # Open up desired q from json file
            # qd = parse_setpoint(nq)
            qd = 3.14/180 * np.array([210.0]).reshape(-1,1)
            tau_d = np.array([30.0]).reshape(-1,1)
            f_e = 0

            L1 = 66.68/1000
            beta1 = 0
            beta2 = -16.43*3.14/180
            y01 = 0
            y02 = 1.17/1000
            z01 = 33.1/2000
            z02 = -33.1/2000
            y11 = L1
            y12 = 66.81/1000
            L2 = (y12 - y02)/np.cos(beta2)
            # qd = np.zeros((12,1)).reshape(-1,1)
            # print(f"[DEBUG] Our desired config: {qd}\n")
            # print(f"[DEBUG] Our error qd - q: {qd - q}")
            zero =  np.zeros((nq,1))
            t_old = time.time()
            # our loop's "starting" time
            t_0 = time.time()
            while 1:
                if kbhit():
                    key_input = getch()
                    # c = getch()
                    first_time = True
                    Joints.send_torque_cmd(nq * [0])
                    ser.close()
                    print("[Q KEY PRESSED] : All motors stopped\n")
                    save_data(q_data, q2_data, qd, tau_data, F_model_data, F_fb_data, timestamps, dt_loop)
                    break
                else:
                    t = time.time()
                    q = np.array(Joints.get_position()).reshape(-1,1)
                    # print(f"[DEBUG] dt q2: {t - time.time()}\n")  

                    
                    ser.flushInput()
                    q2 = np.array([3.14/180*float(ser.readline().decode().strip())]).reshape(-1,1)
                    q2_fb = - (q2 - np.pi )

                    # print(f"[DEBUG] q1, q2: {(q)  * 180/3.14, q2_fb * 180/3.14 + 180}\n")
                    # print(f"[DEBUG] q1, q2: {q-np.pi, q2_fb}\n")
                    # mj0 = joint_th[0] - base_offset
                    # mj1 = joint_th[1] - offset1
                    # mj2 = joint_th[2] - offset2

                    q_data=np.append(q_data, q, axis = 1) 
                    q2_data=np.append(q2_data, q2, axis = 1) 

                    
                    # At the first iteration velocity is 0  
                    if first_time:
                        dq = np.zeros((nq,1))
                        first_time = False
                        dq2 = np.zeros((1,1))
                    else:
                        t = time.time()
                        time_elapsed = t-t_0
                        timestamps = np.append(timestamps, time_elapsed) 
                        dt = t - t_old
                        # print(f"[DEBUG] dt: {dt}\n")  
                        t_old = t
                        dq = diff(q, q_old, dt)
                        dq2 = diff(q2_fb,q2_old, dt)
                    # calculate errors
                    err = q - qd
                    err_dot = dq
                    # print(f"[DEBUG] q: {q}\n")
                    # TODO: set position limits
                    # tau = turtle_controller(q,dq,qd,zero,zero,Kp,KD)
                    # tau_data=np.append(tau_data, tau, axis=1) 
                    # print(f"[DEBUG] q: {q * 180/3.14}\n")
                    # print(f"[DEBUG] qd: {qd * 180/3.14}\n")
                    # print(f"[DEBUG] e: {err * 180/3.14}\n")
                    q_old = q
                    q2_old = q2_fb
                    # input = grab_arm_current(tau, min_torque, max_torque)
                    
                    
                    Fk = -63.8 * (q-np.pi - q2_fb)
                    Fkd = 63.8 * (qd-np.pi - q2_fb)
                    # Distance
                    # k = 45196.8912
                    # Fk = -k*((L2*cos(beta2)*(sin(q2_fb) - sin(beta2)**2*sin(q2_fb)) - L2*cos(beta2)*sin(beta2)**2*sin(q2_fb))*(y01 - y02) + (L2*sin(beta2)*(sin(q2_fb) - cos(beta2)**2*sin(q2_fb)) - L2*cos(beta2)**2*sin(beta2)*sin(q2_fb))*(z01 - z02) + (2*(L2*cos(beta2)*(sin(q2_fb) - sin(beta2)**2*sin(q2_fb)) - L2*cos(beta2)*sin(beta2)**2*sin(q2_fb))*(L1*cos(beta1)*(cos(q) - sin(beta1)**2*(cos(q) - 1)) - L2*cos(beta2)*(cos(q2_fb) - sin(beta2)**2*(cos(q2_fb) - 1)) - L1*cos(beta1)*sin(beta1)**2*(cos(q) - 1) + L2*cos(beta2)*sin(beta2)**2*(cos(q2_fb) - 1)))/3 + (2*(L2*sin(beta2)*(sin(q2_fb) - cos(beta2)**2*sin(q2_fb)) - L2*cos(beta2)**2*sin(beta2)*sin(q2_fb))*(L1*sin(beta1)*(cos(q) - cos(beta1)**2*(cos(q) - 1)) - L2*sin(beta2)*(cos(q2_fb) - cos(beta2)**2*(cos(q2_fb) - 1)) - L1*cos(beta1)**2*sin(beta1)*(cos(q) - 1) + L2*cos(beta2)**2*sin(beta2)*(cos(q2_fb) - 1)))/3 - (2*(L2*cos(beta2)**2*cos(q2_fb) - L2*cos(q2_fb)*sin(beta2)**2)*(L1*sin(q)*cos(beta1)**2 - L2*sin(q2_fb)*cos(beta2)**2 - L1*sin(q)*sin(beta1)**2 + L2*sin(q2_fb)*sin(beta2)**2))/3)/2
                    # q2_fb = qd-np.pi
                    # Fkd = (k*((L1*cos(beta1)*(sin(qd - np.pi) - sin(beta1)**2*sin(qd - np.pi)) - L1*cos(beta1)*sin(beta1)**2*sin(qd - np.pi))*(y01 - y02) + (L1*sin(beta1)*(sin(qd - np.pi) - cos(beta1)**2*sin(qd - np.pi)) - L1*cos(beta1)**2*sin(beta1)*sin(qd - np.pi))*(z01 - z02) + (2*(L1*cos(beta1)*(sin(qd - np.pi) - sin(beta1)**2*sin(qd - np.pi)) - L1*cos(beta1)*sin(beta1)**2*sin(qd - np.pi))*(L1*cos(beta1)*(cos(qd - np.pi) - sin(beta1)**2*(cos(qd - np.pi) - 1)) - L2*cos(beta2)*(cos(q2_fb) - sin(beta2)**2*(cos(q2) - 1)) - L1*cos(beta1)*sin(beta1)**2*(cos(qd - np.pi) - 1) + L2*cos(beta2)*sin(beta2)**2*(cos(q2) - 1)))/3 + (2*(L1*sin(beta1)*(sin(qd - np.pi) - cos(beta1)**2*sin(qd - np.pi)) - L1*cos(beta1)**2*sin(beta1)*sin(qd - np.pi))*(L1*sin(beta1)*(cos(qd - np.pi) - cos(beta1)**2*(cos(qd - np.pi) - 1)) - L2*sin(beta2)*(cos(q2) - cos(beta2)**2*(cos(q2) - 1)) - L1*cos(beta1)**2*sin(beta1)*(cos(qd - np.pi) - 1) + L2*cos(beta2)**2*sin(beta2)*(cos(q2) - 1)))/3 - (2*(L1*cos(beta1)**2*cos(qd - np.pi) - L1*cos(qd - np.pi)*sin(beta1)**2)*(L1*sin(qd - np.pi)*cos(beta1)**2 - L2*sin(q2)*cos(beta2)**2 - L1*sin(qd - np.pi)*sin(beta1)**2 + L2*sin(q2)*sin(beta2)**2))/3))/2
                    
                    # Neo Hookean
                    # dtheta = (qd-np.pi - q2_fb)
                    # L1 = 66.68/1000
                    # w = 0.08/2
                    # Px = L1*np.sin(dtheta)
                    # alpha = np.arctan(Px/w)
                    # ell_1 = w/np.cos(alpha)
                    # lam = ell_1/w
                    # Fkd = (lam[0] - 1/lam[0]**3)
                    # Fkd = 63.8 * 1/76.6 * 63.8 * (qd-np.pi - q2_fb)
                    tau = 100*((qd-q)) + 15*(- dq) + Fkd

                    if ft_sensor:
                        f_data = conn.recv(1024)
                        # print(f_data)
                        if not f_data:
                            F = F
                        else:
                            if f_data.decode().count("\r") > 1:
                                idx = f_data.decode().index("\r")
                                idx2 = find_nth(f_data.decode(), "\r", 2)
                                # print(f_data)
                                # print(idx)
                                f_string = f_data.decode()[idx+2:idx2]
                                # print(f_string)
                                F = f_string.split(", ")
                                # print(F)
                            else:
                                F = f_data.decode().strip().split(", ")
                                # print(F)
                        F = list(map(float, F))
                        Fz = -F[2]
                        
                        F_fb_data=np.append(F_fb_data, np.array(Fz).reshape(-1,1), axis=1) 
                        F_fb_data = uniform_filter1d(F_fb_data,size=10)
                        print(f"[DEBUG] Fz: {F_fb_data[0][-1]*7.1073}\n")
                        # Kd = 7 for pure feedback
                        # tau = 60*((qd-q)) + 12*(- dq)
                        # 10 and 12.5 work decent
                        tau = (2*(F_fb_data[0][-1]*7.1073 - tau_d) + (Fkd) - 15*dq + 10*f_e)
                        f_e = (F_fb_data[0][-1]*7.1073 - tau_d)*0.01 + f_e
                        tau = (5*(Fk - tau_d) + (Fkd) - 15*dq + 5*f_e)
                        f_e = (Fk - tau_d)*0.01 + f_e
                    else:
                        tau = (3*(Fk - tau_d) + (Fkd) - 15*dq + 5*f_e)
                        f_e = (Fk - tau_d)*0.01 + f_e
                    # F_model_data=np.append(F_model_data, np.array(Fk).reshape(-1,1), axis=1) 

                    input_history = np.append(input_history[:,1:], tau,axis=1)

                    input_mean = np.mean(input_history, axis = 1)

                    input = grab_arm_current(input_mean, min_torque, max_torque)
                    # print(f"[DEBUG] q1, q2: {q-np.pi, q2_fb}\n")
                    print(f"[DEBUG] Torque on joint: {Fkd}\n") 
                    
                    
                    Joints.send_torque_cmd(input)
                    # print(f"[DEBUG] joint cmds: {tau}\n")
                    tau_data=np.append(tau_data, tau, axis=1) 
        
        elif key_input == 'n':
            # Update to new config
            m, l, config_params = parse_config()

    print("[END OF PROGRAM] Disabling torque\n")
    # Disable Dynamixel Torque
    Joints.disable_torque()
    # Close port
    # portHandlerMod.closePort()
except Exception:
    print("[ERROR] Disabling torque\n")
    Joints.disable_torque()
    s.close()
    traceback.print_exc()