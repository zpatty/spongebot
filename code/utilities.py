#!/usr/bin/env python
# -*- coding: utf-8 -*-
from datetime import datetime
import os
import numpy as np
import scipy
import json
import time
from dyn_functions import *
from scipy.ndimage import uniform_filter1d


def save_data(q_data, qd, tau_data, input_data, c_data, t_0, timestamps, config_params, qd_params, dt_loop, traj = False, x_data = None, F_fb_data = None):

    print(f"time since: {time.time() - t_0}\n")
    t = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    folder_name =  "data/" + t
    os.makedirs(folder_name, exist_ok=True)
    if F_fb_data is None:
        scipy.io.savemat(folder_name + "/data.mat", {'q_data': q_data.T,'tau_data': tau_data.T,'time_data': timestamps,'q_desired': qd,'c_data': c_data.T,'input_data':input_data.T})
    else:
        scipy.io.savemat(folder_name + "/data.mat", {'q_data': q_data.T,'tau_data': tau_data.T,'time_data': timestamps,'q_desired': qd,'c_data': c_data.T,'input_data':input_data.T,'F_fb_data':F_fb_data.T})
    new_config = folder_name + "/config.json"
    with open(new_config, "w") as outfile:
        outfile.write(config_params)
    new_config = folder_name + "/config.json"
    with open(new_config, "w") as outfile:
        outfile.write(config_params)
                # Writing to new config.json
    if not traj:
        qd_config = folder_name + "/qd.json"
        with open(qd_config, "w") as outfile:
            outfile.write(qd_params)
    

def parse_config():
    with open('config.json') as config:
        param = json.load(config)
    print(f"[MESSAGE] Config: {param}\n")
    # mm = param['mm']
    # mm = param['mm']
    # hm = param['hm']
    # rm = param['rm']
    # N  = param['N']
    # kb = param['kb']
    # ks = param['ks']
    # bb = param['bb']
    # bs = param['bs']
    # e  = param['e']
    # kp = param['kp']
    # kl = param['kl']
    # kp_bot = param['kp_bot']
    # kl_bot = param['kl_bot']
    # km = param['km']
    # k_base = param['k_base']
    # kp_bot  = param['kp_bot']
    # km_bottom = param['km_bottom']
    # KD_mod = param['KD_mod']
    # KD_m = param['KD_m']
    # kc = param['kc']
    # Ka = param['Ka']
    # c_offset = param['c_offset']
    # contact = param['contact']          
    
    # Serializing json
    config_params = json.dumps(param, indent=14)
    
    return param, config_params

def parse_setpoint():
    with open('q.json') as q_json:
        qd_str = json.load(q_json)

    # k1 = param['k1']
    # k2 = param['k2']
    # k3 = param['k3']
    # k4 = param['k4']
    # phi1 = param['phi1']
    # phi2 = param['phi2']
    # phi3 = param['phi3']
    # phi4 = param['phi4']
    # dL1 = param['dL1']
    # dL2 = param['dL2']
    # dL3 = param['dL3']
    # dL4 = param['dL4'] 
    # jm0 = param['jm0']
    # jm1 = param['jm1']
    # jm2 = param['jm2']
    # jm3 = param['jm3']
    # qd = np.zeros((nq,1))
    # for i in range(nq):
    #     qd[i] = ['q' + str(i)]
        
    qd_params = json.dumps(qd_str, indent=14)
    
    return qd_str, qd_params

def torque_to_current(tau,l):
    # print(f"[DEBUG] OG TAU: {tau}\n")
    tau_clip = np.concatenate((np.zeros((1,1)), tau[1:4], np.zeros((1,1)), tau[5:8],np.zeros((1,1)), tau[9:12], np.zeros((1,1)), tau[13:]))
    tau_cables = np.maximum(tau_clip,-30 * np.ones((16,1)))
    # put back og joint torque values since we didn't want to clip those
    tau_cables[0,0] = tau[0,0]
    tau_cables[4,0] = tau[4,0]
    tau_cables[8,0] = tau[8,0]
    tau_cables[12,0] = tau[12,0]
    # print(f"[DEBUG] tau cables: {tau_cables}\n")
    arm_input = grab_arm_current(tau_cables, min_torque, max_torque)
    # print("[DEBUG] dt time:", (time.time() - tt))
    # print(f"[DEBUG] arm_input before mod clip: {arm_input}\n")  
    mod_clip =  arm_input[1:4] +  arm_input[5:8] + arm_input[9:12] + arm_input[13:]
    # print(f"mod clip: {mod_clip}\n")
    for mod in range(len(limits)):
        for cable in range(len(limits[0])):
            idx = (mod * 3) + cable
            idx2 = (mod * 4) + cable
            # for cases where we try to extend the cables
            # mod_clip[idx] = 0
            # if mod != 3:
                # mod_clip[idx] = 0
            if mod_clip[idx] < 0:
                if l[mod][cable] >= limits[mod][cable]:
                    mod_clip[idx] = 0
                    # arm_input[idx2] = 0
                # if c[mod][0] < 0:
                #     mod_clip[idx] = 0
    # print(f"[DEBUG] arm_input: {arm_input}\n")                
    # send current command to motors
    mod_cmds = mod_clip
    # print(f"[DEBUG] mod cmds: {mod_cmds}\n")

    return arm_input, mod_cmds

def np_mat_type(dim, element_type=np.float64):
    return np.ctypeslib.ndpointer(dtype=element_type, shape=dim, ndim = 1, flags="C_CONTIGUOUS")

def puppet_controller_wrapper(q,dq,qd,dqd,ddqd,d,hp,mplate,r,s,param,puppet_controller_c,Lm=Lm):
    zero =  np.zeros((len(q),1))
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
    # Kp = np.diag([k_base,kp_bot,kp_bot,kl_bot,km_bottom,kp,kp,kl,km_bottom,kp,kp,kl,km,kp,kp,kl])
    Kp = np.diag([k_base,kp,kp,kl,km_bottom,kp,kp,kl,km_bottom,kp,kp,kl,km,kp,kp,kl])
    KD = np.diag([KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod])
    Kpvec = np.reshape(Kp.astype(float),(256,1))
    KDvec = np.reshape(KD.astype(float),(256,1))
    tau = np.zeros((16,1))
    cont = np.zeros((4,1))
    # print(f"KD: {KD}\n")
    # print(f"[DEBUG] make arrays: {time.time() - t_test}\n") 
    t_test = time.time()
    puppet_controller_c(np.squeeze(q), np.squeeze(dq), np.squeeze(qd),  np.squeeze(zero), np.squeeze(zero), d, mass_module, \
                        mm, hm, rm, r, kb, ks, bb, bs, s, np.squeeze(Kpvec), np.squeeze(KDvec), kc, Ka, c_offset, contact, \
                            2.0, 225.0, np.squeeze(tau), np.squeeze(cont))
    return tau, cont


def impedance_controller_wrapper(q,dq,qd,dqd,ddqd,xd,dxd,dxr,d,hp,mplate,r,s,param,puppet_controller_impedance,Lm=Lm):
    zero =  np.zeros((len(q),1))
    mm = param['mm']
    mm = param['mm']
    hm = param['hm']
    rm = param['rm']
    N  = param['N']
    kb = param['kb']
    ks = param['ks']
    bb = param['bb']
    bs = param['bs']
    bm = param['bm']
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
    Kpx = param['Kpx']
    KDx = param['KDx']
    kc = param['kc']
    Ka = param['Ka']
    c_offset = param['c_offset']
    contact = param['contact']  
    t_test = time.time()
    # Kp = np.diag([k_base,kp_bot,kp_bot,kl_bot,km_bottom,kp,kp,kl,km_bottom,kp,kp,kl,km,kp,kp,kl])
    Kp = np.diag([k_base,kp,kp,kl,km_bottom,kp,kp,kl,km_bottom,kp,kp,kl,km,kp,kp,kl])
    KD = np.diag([KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod,KD_m,KD_mod,KD_mod,KD_mod])
    Kpvec = np.reshape(Kp.astype(float),(256,1))
    KDvec = np.reshape(KD.astype(float),(256,1))
    tau = np.zeros((16,1))
    tau_r = np.zeros((16,1))
    x = np.zeros((3,1))
    cont = np.zeros((4,1))
    lam = np.zeros((16,1))
    lam_vec = np.reshape(lam.astype(float),(16,1))
    # print(f"KD: {KD}\n")
    # print(f"[DEBUG] make arrays: {time.time() - t_test}\n") 
    t_test = time.time()
    # print(f"[DEBUG] Our desired config: {np.squeeze(xd)}\n")
    puppet_controller_impedance(np.squeeze(q), np.squeeze(dq), np.squeeze(qd),  np.squeeze(zero), np.squeeze(zero), d, mass_module, \
                        mm, hm, rm, r, kb, ks, bb, bs, bm, s, np.squeeze(Kpvec), np.squeeze(KDvec), Kpx, KDx, np.squeeze(xd), np.squeeze(dxd), np.squeeze(dxr), kc, Ka, c_offset, contact, \
                            2.0, 225.0, np.squeeze(tau), np.squeeze(tau_r), np.squeeze(x), np.squeeze(cont))
    return tau, tau_r, x, cont


def find_nth(haystack, needle, n):
    start = haystack.find(needle)
    while start >= 0 and n > 1:
        start = haystack.find(needle, start+len(needle))
        n -= 1
    return start