import numpy as np
from numpy import log, exp
from MC import MC
from puppet_controller import puppet_controller
from dyn_functions import *

# q = 0.01*np.ones((12,1))
q = grab_qd(0,0,0,0,0,0, 0, 0, 0, 0, 0, 0)
print(f"q: {q}")
dq = np.zeros((12,1))
# qd = np.zeros((12,1))
# actuating motor 3 entirely
# qd = grab_qd(4.5,4.5,4.5,1.5708, 1.5708, 1.5708, 0, 0, 0, 0, 0, 0)
# print(f"Our qd: {qd}\n")
# actuating motor 1 entirely
# qd = grab_qd(1.9,1.9,1.9,0,0,0, 0, 0, 0, 0, 0, 0)
# qd = np.array([ 0.        ,
#               -0.24910206,
#               -0.12554113,
#               -0.107     ,
#                0.        ,
#               -0.24910206,
#               -0.12554113,
#               -0.107     ,
#                0.        ,
#               -0.24910206,
#               -0.12554113,
#               -0.107     ]).reshape(-1,1)
# actuating motor 2 entirely:
qd = np.array([ 3.        ,
               0.24981502,
              -0.125     ,
              -0.10666667,
               2.        ,
               0.24981502,
              -0.125     ,
              -0.10666667,
               1.        ,
               0.24981502,
              -0.125     ,
              -0.10666667]).reshape(-1,1)
# qd = grab_qd(0,0,0,0,0,0, 0, 0, 0, 0, 0.3, 0.3)

dqd = np.zeros((12,1))
# ddqd = np.array([0,
#    0.051869021363341,
#    0.080781214541558,
#   -0.480000000000000,
#   -4.800000000000001,
#    0.051869021363341,
#    0.080781214541558,
#                    0,
#    2.400000000000000,
#   -0.051869021363341,
#   -0.080781214541558,
#   -0.720000000000000]).reshape(-1,1)
ddqd = np.zeros((12,1))
L0 = 0.8
d = 0.04
N = 6
r = 0.007
rm = 0.02
hm = 0.06
mm = 0.08
kb = 1
ks = 2000
bb = 1
bs = 1
e = 1/1000
m = 0.17
kp = 10
km = 10
Kp = np.diag([25,kp,kp,kp,km,kp,kp,kp,km,kp,kp,kp])
KD = 0.5
hp = 0.007
kc = 10000
tau = puppet_controller(q,dq,qd,dqd,ddqd,d,hp,m,mm,hm,rm,r,N,kb,ks,bb,bs,L0,e,Kp,KD,kc)
print(f"motor commands: {tau}\n")
tau_clip = np.concatenate((np.zeros((1,1)), tau[1:4], np.zeros((1,1)), tau[5:8],np.zeros((1,1)), tau[9:]))
print(f"tau clipped: {tau_clip}\n")
tau_cables = np.maximum(3 * tau_clip.reshape(1,-1),-30 * np.ones((1,12)))
tau_cables[0,0] = tau[0,0]
tau_cables[0,4] = tau[4,0]
tau_cables[0,8] = tau[8,0]

print(f"tau cables: {tau_cables.reshape(-1,1)}\n")