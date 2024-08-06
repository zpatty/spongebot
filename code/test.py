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
from FourModController import *                # Controller 
from Constants import *                        # File of constant variables
from Mod import *
import json
import traceback
from utilities import *
import ctypes
from ctypes import *

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
    

ctrl = ctypes.CDLL("puppet_controller_4_cg/puppet_controller_4_cg.so")
puppet_controller_c = ctrl.puppet_controller_4_cg
puppet_controller_c.restype = None
puppet_controller_c.argtypes = np_mat_type(16), np_mat_type(16), np_mat_type(16), np_mat_type(16), np_mat_type(16),\
c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_double,\
np_mat_type(256), np_mat_type(256), c_double, c_double, c_double, c_double, c_double, c_double, np_mat_type(16)
# MAKE SURE TO UNCOMMENT THIS WHEN YOURE DONE

# the motor angles of the 9 motors at neutral position 
t_old = time.time()
#th0 = th0
# t_old = time.time()
print("[DEBUG] dt:", (time.time() - t_old))  
#th1_0 = th0[:3]
#th2_0 = th0[3:6]
#th3_0 = th0[6:9]
#th4_0 = th0[9:]

th0 = [th1_0, th2_0, th3_0, th4_0]

# offset1 = joint_th0[1]
offset1 = 4.428602534625846
# offset2 = joint_th0[2]
offset2 = 2.2089323345553233
# offset3 = joint_th0[3]
offset3 = 3.172272269347506 + 0.558*0
base_offset = 4.17
# base_offset = joint_th0[0]
print(f"Offset for XM430s: {base_offset, offset1, offset2, offset3}")

print(f"Mod 1 Reference Angles: {th1_0}\n")
print(f"Mod 2 Reference Angles: {th2_0}\n")
print(f"Mod 3 Reference Angles: {th3_0}\n")
print(f"Mod 4 Reference Angles: {th4_0}\n")

print(f"Current theta readings: {th0}\n")

BigMotors = [0,0,0,0]
# our max arc length (in m)
s = (l4_0[0] + l4_0[1] + l4_0[2])/3
# cables of the three mods at neutral position 
l0 = [l1_0, l2_0, l3_0, l4_0]
# grab cable lengths of all 9 motors
l = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]]
l = grab_arm_cable_lens(th1_0 + th2_0 + th3_0 + th4_0, l, l0, th0, r)
print(f"Cable lengths based off of dtheta: {l}\n")
print(f" big motors angles: {BigMotors}")
q = grab_arm_q(l[0], l[1], l[2], l[3], BigMotors[0], BigMotors[1], BigMotors[2], BigMotors[3], s, d)

nq = 16
nmod = 4
nm = 4
q_data = np.zeros((nq,1))
tau_data = np.zeros((nq,1))
timestamps = np.zeros((1,1))
c_data = np.zeros((nmod,1))
dt_loop = np.zeros((1,1))       # hold dt data 

# Report our initial configuration
print(f"Our current q: {q}\n")
first_time = True

np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

            
q_data = np.zeros((nq,1))
tau_data = np.zeros((nq,1))
timestamps = np.zeros((1,1))
c_data = np.zeros((nmod,1))
# Open up controller parameters
param, config_params = parse_config()
# Open up desired q from json file
qd_str, qd_params = parse_setpoint()
qd = grab_qd(qd_str)
# print(f"[DEBUG] Our desired config: {qd}\n")
zero =  np.zeros((16,1))
t_old = time.time()
# our loop's "starting" time
t_0 = time.time()
tvec = mat2np('tvec.mat', 'tvec')
q_old = q
while 1:
    # grab current time
    pos = th1_0 + th2_0 + th3_0 + th4_0
    t_test = time.time()
    l = grab_arm_cable_lens(pos, l, l0, th0, r)     
    # print(f"[DEBUG] cable lengths: {l}\n")  
    joint_th = [0,0,0,0]
    mj0 = joint_th[0] - base_offset
    mj1 = joint_th[1] - offset1
    mj2 = joint_th[2] - offset2
    mj3 = joint_th[3] - offset3
    q = grab_arm_q(l[0], l[1], l[2], l[3], mj0, mj1, mj2, mj3, s, d)
    print(f"[DEBUG] get q: {time.time() - t_test}\n") 
    q_data=np.append(q_data, q, axis = 1) 
    # At the first iteration velocity is 0  

    t = time.time()
    
    time_elapsed = t-t_0
    # n = get_qindex(time_elapsed, tvec)
    # print(f"[DEBUG] get_qindex: {time.time() - t}\n")  
    
    t_test = time.time()
    n_t = np.argmax(tvec>time_elapsed)
    print(f"[DEBUG] get_qindex: {time.time() - t_test}\n")  
    timestamps = np.append(timestamps, time_elapsed) 
    dt = t - t_old
    print(f"[DEBUG] dt: {dt}\n") 
    t_old = t
    dq = diff(q, q_old, dt)
    q_old = q
    # calculate errors
    err = q - qd
    err_dot = dq
    # print(f"Error: {err}\n")
    # print(f"[DEBUG] q: {q}\n")
    # TODO: set position limits
    # t_test = time.time()
    # tau, cont = puppet_controller(q,dq,qd,zero,zero,d,hp,mass_module,r,s,param,Lm=Lm)
    # print(f"[DEBUG] python: {time.time() - t_test}\n")  
# q,dq,qd,dqd,ddqd,d,m,mm,hm,rm,r,kb,ks,bb,bs,L0,Kp,KD,kc,ka,offset,contact) 

    mm = param['mm']
    mm = param['mm']
    hm = param['hm']
    rm = param['rm']
    N  = param['N']
    kb = param['kb']
    ks = param['ks']
    bb = param['bb']
    bs = param['bs']
    e  = param['e']
    kp = param['kp']
    kl = param['kl']
    kp_bot = param['kp_bot']
    kl_bot = param['kl_bot']
    km = param['km']
    k_base = param['k_base']
    kp_bot  = param['kp_bot']
    km_bottom = param['km_bottom']
    KD_mod = param['KD_mod']
    KD_m = param['KD_m']
    kc = param['kc']
    Ka = param['Ka']
    c_offset = param['c_offset']
    contact = param['contact']  
    t_test = time.time()
    Kp = np.diag([k_base,kp_bot,kp_bot,kl_bot,km_bottom,kp,kp,kl,km_bottom,kp,kp,kl,km,kp,kp,kl])
    KD = np.diag([KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod])
    Kpvec = np.reshape(Kp.astype(float),(256,1))
    KDvec = np.reshape(KD.astype(float),(256,1))
    tau_c = np.zeros((16,1))
    print(f"[DEBUG] make arrays: {time.time() - t_test}\n") 
    t_test = time.time()
    # puppet_controller_c(np.squeeze(q), np.squeeze(dq), np.squeeze(qd),  np.squeeze(zero), np.squeeze(zero), d, mass_module, \
    #                     mm, hm, rm, r, kb, ks, bb, bs, s, np.squeeze(Kpvec), np.squeeze(KDvec), kc, Ka, c_offset, contact, \
    #                         200.0, 2.0, np.squeeze(tau_c))
    tau = puppet_controller_wrapper(q,dq,qd,zero,zero,d,hp,mplate,r,s,param,puppet_controller_c,Lm=Lm)
    print(f"[DEBUG] ctypes: {time.time() - t_test}\n") 
    # c_data = np.append(c_data, cont, axis=1) 
    tau_data=np.append(tau_data, tau, axis=1) 
    print(f"[DEBUG] tau: {tau.reshape(1,-1)}\n")
    t_test = time.time()
    arm_input, mod_cmds = torque_to_current(tau,l)
    # 0 time
    print(f"[DEBUG] torque_to_current: {time.time() - t_test}\n") 
    joint_cmds = [arm_input[0], -arm_input[4], -arm_input[8], -arm_input[12]]
    # joint_cmds = [0, 0, 0]
    # print(f"[DEBUG] joint cmds: {joint_cmds}\n")
            