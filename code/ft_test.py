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
from FourModController import *                # Controller 
from Constants import *                        # File of constant variables
from Mod import *
import json
import traceback
from utilities import *
from ctypes import *
import socket
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
    


HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 10000  # Port to listen on (non-privileged ports are > 1023)

sock =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen()
conn, addr = sock.accept()
os.system('sudo /home/zach/SRRA/latency_write.sh')

ctrl = CDLL("puppet_controller_4_cg/puppet_controller_4_cg.so")
puppet_controller_c = ctrl.puppet_controller_4_cg
puppet_controller_c.restype = None
puppet_controller_c.argtypes = np_mat_type(16), np_mat_type(16), np_mat_type(16), np_mat_type(16), np_mat_type(16),\
c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double,\
np_mat_type(256), np_mat_type(256), c_double, c_double, c_double, c_double, c_double, c_double, np_mat_type(16), np_mat_type(4)

ctrl = CDLL("puppet_controller_impedance/puppet_controller_4_cg.so")
puppet_controller_impedance = ctrl.puppet_controller_4_cg
puppet_controller_impedance.restype = None
puppet_controller_impedance.argtypes = np_mat_type(16), np_mat_type(16), np_mat_type(16), np_mat_type(16), np_mat_type(16),\
c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double,\
np_mat_type(256), np_mat_type(256), c_double, c_double, np_mat_type(3), np_mat_type(3), np_mat_type(3), c_double, c_double, c_double, c_double, c_double, c_double, np_mat_type(16), np_mat_type(16), np_mat_type(3), np_mat_type(4)

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

packetHandlerMod = PacketHandler(PROTOCOL_VERSION)
packetHandlerJoint = PacketHandler(PROTOCOL_VERSION)

Arm = Mod(packetHandlerMod, portHandlerMod, [1,2,3,4,5,6,7,8,9, 10, 11, 12])

Arm.set_max_velocity(MAX_PROFILE_VELOCITY)
Arm.set_current_cntrl_mode()
Arm.enable_torque()

# Base Motor, Joint1, Joint2
Joints = Mod(packetHandlerJoint, portHandlerJoint, [0, 1, 2, 3])
Joints.set_current_cntrl_mode()
Joints.enable_torque()
# MAKE SURE TO UNCOMMENT THIS WHEN YOURE DONE
# the motor angles of the 9 motors at neutral position 
t_old = time.time()
#th0 = Arm.get_position()
joint_th0 = Joints.get_position()
# t_old = time.time()
print("[DEBUG] dt:", (time.time() - t_old))  
#th1_0 = th0[:3]
#th2_0 = th0[3:6]
#th3_0 = th0[6:9]
#th4_0 = th0[9:]

th0 = calibration


# offset1 = joint_th0[1]
offsets = [4.707787038021033, 4.292078244504024, 2.176718738009725, 2.8716120349219203]
offset1 = offsets[1]
offset2 = offsets[2]
offset3 = offsets[3]
# offset3 = 3.172272269347506 + 0.558*0
base_offset = offsets[0]
# base_offset = joint_th0[0]
print(f"Offset for XM430s: {base_offset, offset1, offset2, offset3}")
BigMotors = [joint_th0[0] - base_offset, joint_th0[1] - offset1, joint_th0[2] - offset2, joint_th0[3] - offset3]

print(f"Mod 1 Reference Angles: {th1_0}\n")
print(f"Mod 2 Reference Angles: {th2_0}\n")
print(f"Mod 3 Reference Angles: {th3_0}\n")
print(f"Mod 4 Reference Angles: {th4_0}\n")

print(f"Current theta readings: {Arm.get_position()}\n")
print(f"Z Motor Position: {BigMotors[0]}")
# our max arc length (in m)
s = (l4_0[0] + l4_0[1] + l4_0[2])/3
# cables of the three mods at neutral position 
l0 = [l1_0, l2_0, l3_0, l4_0]
# grab cable lengths of all 9 motors
l = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]]
l = grab_arm_cable_lens(Arm.get_position(), l, l0, th0, r)
print(f"Cable lengths based off of dtheta: {l}\n")
print(f" big motors angles: {BigMotors}")
q = grab_arm_q(l[0], l[1], l[2], l[3], BigMotors[0], BigMotors[1], BigMotors[2], BigMotors[3], s, d)

nq = 16
nmod = 4
nm = 4
q_data = np.zeros((nq,10))
print(f"Q DATA SIZE: {q_data.shape}")
tau_data = np.zeros((nq,1))
timestamps = np.zeros((1,1))
c_data = np.zeros((nmod,1))
dt_loop = np.zeros((1,1))       # hold dt data 

# Report our initial configuration
print(f"Our current q: {q}\n")
first_time = True

np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

try: 
    while 1:
        # Arm.send_torque_cmd(nmod * [0])
        # Joints.send_torque_cmd(nm * [0])
        print("\nT: Trajectory, W: Set point, C: CALIBRATE FIRST! (or press SPACE to quit!)")
        key_input = getch()
        if key_input == chr(SPACE_ASCII_VALUE):
            print("we're quitting\n")
            break
        elif key_input == chr(UKEY_ASCII_VALUE):
            # Set calibration protocol
            # Module 1 calibration first
            # start with small current and check velocity
            motor_cmds = 12 * [-30]
            Arm.send_torque_cmd(motor_cmds)
            while 1:
                if kbhit():
                    c = getch()
                    if c == chr(QKEY_ASCII_VALUE):
                        first_time = True
                    Arm.send_torque_cmd(12 * [0])
                    Joints.send_torque_cmd(4 * [0])
                    print("[Q KEY PRESSED] : All motors stopped\n")
                    break
                else:
                    print("unwinding...")
        elif key_input == chr(CKEY_ASCII_VALUE):
            th0 = Arm.get_position()
            th1_0 = th0[:3]
            th2_0 = th0[3:6]
            th3_0 = th0[6:9]
            th4_0 = th0[9:]
            th0 = [th1_0, th2_0, th3_0, th4_0]
            print(f"TH0: {th0}")
            # folder_name =  'theta'
            # os.makedirs(folder_name, exist_ok=True)
            # theta_config = folder_name + "/theta_0.json"
            # with open(theta_config, "w") as outfile:
            #     outfile.write(th0)
            # grab cable lengths of all 9 motors
            l = grab_arm_cable_lens(Arm.get_position(), l, l0, th0, r)
            print(f"Cable lengths based off of dtheta: {l}\n")
            q = grab_arm_q(l[0], l[1], l[2], l[3], BigMotors[0], BigMotors[1], BigMotors[2], BigMotors[3], s, d)

            # Report our initial configuration
            print(f"Our current q: {q}\n")
        elif key_input == chr(SKEY_ASCII_VALUE):
            # Set calibration protocol
            motor_cmds = 12 * [0] 
            print("\n choose module to compress")
            key_input = getch()
            if key_input == chr(0x31):
                # Module 1 calibration third 
                motor_cmds[0] = 40
                motor_cmds[1] = 40
                motor_cmds[2] = 40
            elif key_input == chr(0x32):
                # Module 2 calibration second
                motor_cmds[3] = 40
                motor_cmds[4] = 40
                motor_cmds[5] = 40
            elif key_input == chr(0x33):
                print("\nmodule 3\n")
                motor_cmds[6] = 40
                motor_cmds[7] = 40
                motor_cmds[8] = 40
            elif key_input == chr(0x34):
                print("\nmodule 3\n")
                motor_cmds[9] = 40
                motor_cmds[10] = 40
                motor_cmds[11] = 40
            # set things into extended pos mode
            # check when motor starts to hit current--> that will be when 
            # Module 3 calibration first
            # start with small current and check velocity       
            Arm.send_torque_cmd(motor_cmds)
            while 1:
                if kbhit():
                    c = getch()
                    if c == chr(QKEY_ASCII_VALUE):
                        first_time = True
                    Arm.send_torque_cmd(12 * [0])
                    print("[Q KEY PRESSED] : All motors stopped\n")
                    break
                else:
                    print("compressing...")
            
           
        elif key_input == chr(BKEY_ASCII_VALUE):
            Arm.disable_torque()
            Arm.set_velocity_mode()
            Arm.enable_torque()

            if kbhit():
                c = getch()
                Arm.send_vel_cmd(12 * [0])
                print("[Q KEY PRESSED] : All motors stopped\n")
                break
            else:
                
                f = [1]*4
                threshold = 200
                while f[0] and f[1] and f[2] and f[3]:
                    if kbhit():
                        c = getch()
                        Arm.send_vel_cmd(12 * [0])
                        print("[Q KEY PRESSED] : All motors stopped\n")
                        break
                    else:
                        v = np.array([-30]*3*f[0] + [-30]*3*f[1] + [-30]*3*f[2] + [-30]*3*f[3]).astype(int).tolist()
                        Arm.send_vel_cmd(v)
                        time.sleep(1)
                        v = np.array([30]*3*f[0] + [30]*3*f[1] + [30]*3*f[2] + [30]*3*f[3]).astype(int).tolist()
                        Arm.send_vel_cmd(v)
                        time.sleep(0.05)
                        current = Arm.get_current()
                        print(f"Current: {current}\n")  
                        for i in range(3):
                            c = np.mean(current[i*3:i*3+2])
                            if c > threshold:
                                print(f"Homed Module {i}\n")  
                                pos = Arm.get_position()
                                th0[i] = pos[i*3:i*3+2]
                                f[i] = 0
                


            
            
        elif key_input == chr(NKEY_ASCII_VALUE):    # print out the length changes
            
            pos = Arm.get_position()
            l = grab_arm_cable_lens(pos, l, l0, th0, r)     
            # print(f"[DEBUG] cable lengths: {l}\n")  
            joint_th = Joints.get_position()
            mj0 = joint_th[0] - base_offset
            mj1 = joint_th[1] - offset1
            mj2 = joint_th[2] - offset2
            mj3 = joint_th[3] - offset3
            q = grab_arm_q(l[0], l[1], l[2], l[3], mj0, mj1, mj2, mj3, s, d)
            q_data = np.zeros((nq*2, 1))
            print(f"Q DATA SIZE: {q_data.shape}")
            tau_data = np.zeros((nq,1))
            input_data = np.zeros((nq,1))
            timestamps = np.zeros((1,1))
            c_data = np.zeros((nmod,1))
            F_fb_data = np.zeros((1,10))
            # Open up controller parameters
            param, config_params = parse_config()
            # Open up desired q from json file
            qd_str, qd_params = parse_setpoint()
            qd = grab_qd(qd_str)
            qd_mat = mat2np('tube_up/qd.mat', 'qd')
            dqd_mat = mat2np('tube_up/dqd.mat', 'dqd')
            ddqd_mat = mat2np('tube_up/ddqd.mat', 'ddqd')
            tvec = mat2np('tube_up/tvec.mat', 'tvec')
            # print(f"[DEBUG] Our desired config: {qd}\n")
            zero =  np.zeros((16,1))
            t_old = time.time()
            # our loop's "starting" time
            t_0 = time.time()
            while 1:
                if kbhit():
                    c = getch()
                    first_time = True
                    for i in range(10):
                        Arm.send_torque_cmd(12 * [0])
                        Joints.send_torque_cmd(4 * [0])
                        print("[Q KEY PRESSED] : All motors stopped\n")
                    print(f"Q DATA SIZE: {q_data.shape}")
                    print(f"[OUTPUT] Our desired config: {qd}\n")
                    print(f"[OUTPUT] Our last recorded q: {q}\n")
                    print(f"[OUTPUT] Our last recorded c: {cont}\n")
                    print(f"max dt value: {np.max(dt_loop)}\n")
                    print(f"last time: {timestamps[-1]}\n")
                    save_data(q_data, qd, tau_data, input_data, c_data, t_0, timestamps, config_params, qd_params, dt_loop)
                    break
                else:
                    # grab current time
                    pos = Arm.get_position()
                    l = grab_arm_cable_lens(pos, l, l0, th0, r)     
                    # print(f"[DEBUG] cable lengths: {l}\n")  
                    joint_th = Joints.get_position()
                    mj0 = joint_th[0] - base_offset
                    mj1 = joint_th[1] - offset1
                    mj2 = joint_th[2] - offset2
                    mj3 = joint_th[3] - offset3
                    q = grab_arm_q(l[0], l[1], l[2], l[3], mj0, mj1, mj2, mj3, s, d)
                    # print(f"QUEUE SHAPE: {q_data.shape}")
                    # last_ten = q_data[:, -10:]
                    # print(f"QLAST TEN SHAPE: {last_ten.shape}")
                    # q_filtered = uniform_filter1d(last_ten, size=10)
                    # print(f"Q SHAPE: {q.shape}")
                    # q_data=np.append(q_data, q, axis = 1) 
                    
                    # q = q_filtered[:,[5]]
                    if key_input == chr(TKEY_ASCII_VALUE):
                        n = np.argmax(tvec>time.time() - t_0) - 1
                        # print(f"this takes: {time.time() - tt}\n")
                        qd = np.array(qd_mat[:, n]).reshape(-1,1)
                        dqd = np.array(dqd_mat[:, n]).reshape(-1,1)
                        ddqd = np.array(ddqd_mat[:, n]).reshape(-1,1)
                    else:
                        dqd = zero
                        ddqd = zero
                    # At the first iteration velocity is 0  
                    if first_time:
                        dq = np.zeros((nq,1))
                        first_time = False
                    else:
                        t = time.time()
                        time_elapsed = t-t_0
                        timestamps = np.append(timestamps, time_elapsed) 
                        dt = t - t_old
                        print(f"[DEBUG] dt: {dt}\n")  
                        t_old = t
                        dq = diff(q, q_old, dt)
                    q_old = q
                    q_data=np.append(q_data, np.append(q,dq).reshape(-1,1), axis = 1) 
                    # calculate errors
                    err = q - qd
                    err_dot = dq
                    # print(f"Error: {err}\n")
                    # print(f"[DEBUG] q: {q}\n")
                    # TODO: set position limits
                    # tau_test, cont = puppet_controller(q,dq,qd,zero,zero,d,hp,mplate,r,s,param,Lm=Lm)
                    tau, cont = puppet_controller_wrapper(q,dq,qd,dqd,ddqd,d,hp,mplate,d,s,param,puppet_controller_c,Lm=Lm)
                    c_data = np.append(c_data, cont, axis=1) 
                    tau_data=np.append(tau_data, tau, axis=1) 
                    print(f"[DEBUG] tau: {tau}\n")
                    tau = np.zeros((16,1))
                    arm_input, mod_cmds = torque_to_current(tau,l)
                    # 0 time
                    last_mod = [0]*12
                    last_mod[-3:] = mod_cmds[-3:]
                    print(f"[DEBUG] last_mod: {mod_cmds}\n")
                    Arm.send_torque_cmd(mod_cmds)
                    
                    joint_cmds = [arm_input[0], -arm_input[4], -arm_input[8], -arm_input[12]]
                    # joint_cmds = [0, 0, 0, 0]
                    # print(f"[DEBUG] joint cmds: {arm_input}\n")
                    Joints.send_torque_cmd(joint_cmds)
                    input_data=np.append(input_data, np.array(arm_input).reshape(-1,1), axis=1) 
                    
        #             q_old = q
        #             err = q - qd
        #             err_dot = dq
        #             tau, cont = puppet_controller(q,dq,qd,dqd,ddqd,d,hp,mplate,r,s,param,Lm=Lm)
        #             c_data = np.append(c_data, cont, axis=1) 
        #             tau_data=np.append(tau_data, tau, axis=1) 
                    
        #             arm_input, mod_cmds = torque_to_current(tau,l)

        #             Arm.send_torque_cmd(mod_cmds)
        #             joint_cmds = [arm_input[0], -arm_input[4], -arm_input[8], -arm_input[12]]
        #             Joints.send_torque_cmd(joint_cmds)
        elif key_input == chr(OKEY_ASCII_VALUE) or key_input == chr(TKEY_ASCII_VALUE) or key_input == chr(WKEY_ASCII_VALUE):    # print out the length changes
            
            pos = Arm.get_position()
            l = grab_arm_cable_lens(pos, l, l0, th0, r)     
            # print(f"[DEBUG] cable lengths: {l}\n")  
            joint_th = Joints.get_position()
            mj0 = joint_th[0] - base_offset
            mj1 = joint_th[1] - offset1
            mj2 = joint_th[2] - offset2
            mj3 = joint_th[3] - offset3
            q = grab_arm_q(l[0], l[1], l[2], l[3], mj0, mj1, mj2, mj3, s, d)
            q_data = np.zeros((nq*2, 1))
            print(f"Q DATA SIZE: {q_data.shape}")
            tau_data = np.zeros((nq,1))
            input_data = np.zeros((nq,1))
            timestamps = np.zeros((1,1))
            c_data = np.zeros((nmod,1))
            x_data = np.zeros((6,1))
            F_fb_data = np.zeros((1,10))
            # Open up controller parameters
            param, config_params = parse_config()
            # Open up desired q from json file
            qd_str, qd_params = parse_setpoint()
            qd = grab_qd(qd_str)
            xd = np.array([qd_str['xd'], qd_str['yd'], qd_str['zd']]).reshape(-1,1)
            dxd = np.zeros((3,1))
            qd_mat = mat2np('qd.mat', 'qd')
            dqd_mat = mat2np('dqd.mat', 'dqd')
            ddqd_mat = mat2np('ddqd.mat', 'ddqd')
            tvec = mat2np('tvec.mat', 'tvec')
            # print(f"[DEBUG] Our desired config: {qd}\n")
            zero =  np.zeros((16,1))
            t_old = time.time()
            # our loop's "starting" time
            t_0 = time.time()
            dqd = zero
            ddqd = zero
            second_time = True
            while 1:
                if kbhit():
                    c = getch()
                    first_time = True
                    for i in range(10):
                        Arm.send_torque_cmd(12 * [0])
                        Joints.send_torque_cmd(4 * [0])
                        print("[Q KEY PRESSED] : All motors stopped\n")
                    print(f"Q DATA SIZE: {q_data.shape}")
                    print(f"[OUTPUT] Our desired config: {qd}\n")
                    print(f"[OUTPUT] Our last recorded q: {q}\n")
                    print(f"[OUTPUT] Our last recorded c: {cont}\n")
                    print(f"max dt value: {np.max(dt_loop)}\n")
                    print(f"last time: {timestamps[-1]}\n")
                    save_data(q_data, qd, tau_data, input_data, c_data, t_0, timestamps, config_params, qd_params, dt_loop, x_data=x_data, F_fb_data=F_fb_data)
                    break
                else:
                    # grab current time
                    pos = Arm.get_position()
                    l = grab_arm_cable_lens(pos, l, l0, th0, r)     
                    # print(f"[DEBUG] cable lengths: {l}\n")  
                    joint_th = Joints.get_position()
                    mj0 = joint_th[0] - base_offset
                    mj1 = joint_th[1] - offset1
                    mj2 = joint_th[2] - offset2
                    mj3 = joint_th[3] - offset3
                    q = grab_arm_q(l[0], l[1], l[2], l[3], mj0, mj1, mj2, mj3, s, d)
                    if key_input == chr(TKEY_ASCII_VALUE):
                        n = np.argmax(tvec>time.time() - t_0) - 1
                        # print(f"this takes: {time.time() - tt}\n")
                        qd = np.array(qd_mat[:, n]).reshape(-1,1)
                        dqd = np.array(dqd_mat[:, n]).reshape(-1,1)
                        ddqd = np.array(ddqd_mat[:, n]).reshape(-1,1)
                    else:
                        dqd = zero
                        ddqd = zero
                    # At the first iteration velocity is 0  
                    if first_time:
                        dq = np.zeros((nq,1))
                        dx = np.zeros((3,1))
                        first_time = False
                        x = np.zeros((3,1))
                    else:
                        if second_time:
                            dx = np.zeros((3,1))
                        else:
                            dx = diff(x, x_old, dt)
                        t = time.time()
                        time_elapsed = t-t_0
                        timestamps = np.append(timestamps, time_elapsed) 
                        dt = t - t_old
                        print(f"[DEBUG] dt: {dt}\n")  
                        t_old = t
                        # print(f"[DEBUG] dq*t: {q-q_old}\n")
                        dq = diff(q, q_old, dt)
                    q_old = q
                    x_old = x
                    q_data=np.append(q_data, np.append(q,dq).reshape(-1,1), axis = 1) 
                    x_data=np.append(x_data, np.append(x,dx).reshape(-1,1), axis = 1) 
                    # calculate errors
                    err = q - qd
                    err_dot = dq
                    # print(f"Error: {err}\n")
                    # print(f"[DEBUG] q: {q}\n")
                    # TODO: set position limits
                    # tau_test, cont = puppet_controller(q,dq,qd,zero,zero,d,hp,mplate,r,s,param,Lm=Lm)
                    tau, tau_r, x, cont = impedance_controller_wrapper(q,dq,qd,dqd,ddqd,xd,dxd,dx,d,hp,mplate,d,s,param,puppet_controller_impedance,Lm=Lm)
                    c_data = np.append(c_data, cont, axis=1) 
                    if key_input == chr(OKEY_ASCII_VALUE):
                        tau = tau_r
                    tau_data=np.append(tau_data, tau, axis=1) 
                    print(f"[DEBUG] x: {x}\n")
                    # print(f"[DEBUG] dq: {dq}\n")
                    # print(f"[DEBUG] lam: {lam}\n")
                    # print(f"[DEBUG] tau, tau_r: {np.linalg.norm(tau),np.linalg.norm(tau_r)}\n")
                    # print(f"[DEBUG] tau_r: {tau_r}\n")
                    # tau = np.zeros((16,1))
                    arm_input, mod_cmds = torque_to_current(tau,l)
                    # 0 time
                    last_mod = [0]*12
                    last_mod[-3:] = mod_cmds[-3:]
                    # print(f"[DEBUG] last_mod: {mod_cmds}\n")
                    first_mod = [0]*12
                    first_mod[0:3] = mod_cmds[0:3]
                    Arm.send_torque_cmd(mod_cmds)
                    
                    joint_cmds = [arm_input[0], -arm_input[4], -arm_input[8], -arm_input[12]]
                    # joint_cmds = [0, 0, 0, 0]
                    # print(f"[DEBUG] joint cmds: {arm_input}\n")
                    Joints.send_torque_cmd(joint_cmds)
                    input_data=np.append(input_data, np.array(arm_input).reshape(-1,1), axis=1) 
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
                    # F_fb_data = uniform_filter1d(F_fb_data,size=10)
                    print(f"[DEBUG] Fz: {F_fb_data[0][-1]}\n")
        
        elif key_input == chr(NKEY_ASCII_VALUE):
            # Update to new config
            with open('config.json') as config:
                param = json.load(config)
            print(param)
            k1 = param['k1']
            k2 = param['k2']
            k3 = param['k3']
            phi1 = param['phi1']
            phi2 = param['phi2']
            phi3 = param['phi3']
            dL1 = param['dL1']
            dL2 = param['dL2']
            dL3 = param['dL3'] 
            jm0 = param['jm0']
            jm1 = param['jm1']
            jm2 = param['jm2']

    print("[END OF PROGRAM] Disabling torque\n")
    # Disable Dynamixel Torque
    Arm.disable_torque()
    Joints.disable_torque()
    # Close port
    # portHandlerMod.closePort()
except Exception:
    print("[ERROR] Disabling torque\n")
    Arm.disable_torque()
    Joints.disable_torque()
    sock.close()
    traceback.print_exc()