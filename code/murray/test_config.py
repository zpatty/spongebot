import os
import math
import json
from datetime import datetime
from dyn_functions import *                    # Dynamixel support functions
from Constants import *

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

while 1:
    print("\nPress S to set your qd or W to run your trajectory!(or press SPACE to quit!)")
    key_input = getch()
    if key_input == chr(SPACE_ASCII_VALUE):
            print("we're quitting\n")
            break
    elif key_input == chr(SKEY_ASCII_VALUE):
        print(f"this is k1: {k1}\n")
        print(f"type for k1: {type(k1)}\n")
    elif key_input == chr(WKEY_ASCII_VALUE):
        with open('config.json') as config:
            param = json.load(config)
        print(param)
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
        km = param['km']
        KD = param['KD']
        kc = param['kc']
        # Serializing json
        new_params = json.dumps(param, indent=14)
        # Writing to new config.json
        t = datetime.now().strftime("%H_%M_%S")
        new_config = "config/" + t + "_config.json"
        with open(new_config, "w") as outfile:
            outfile.write(new_params)
    elif key_input == chr(NKEY_ASCII_VALUE):
        
        with open('q.json') as q_json:
            param = json.load(q_json)
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
        
        qd = grab_qd(k1,k2,k3,phi1, phi2, phi3, dL1, dL2, dL3, jm0, jm1, jm2)
        print(f"qd: {qd}\n")



        # # Serializing json
        # new_q = json.dumps(param, indent=12)
        # # Writing to new config.json
        # t = datetime.now().strftime("%H_%M_%S")
        # q_file = "q/" + t + "_q.json"
        # with open(q_file, "w") as outfile:
        #     outfile.write(new_q)
