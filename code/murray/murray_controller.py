#######################################################
#     Controller for a single pushpuppet segment
#######################################################
# This controller takes basic dynamics and computed torque control law from MLS textbook.
# Paramters:
# (q = configuration, dq = velocity, qd = desired configuration, dqd = desired velocity, ddqd = desired acceleration, 
# d = radius from center to cable, hp = thickness of plastic plates, m = mass of a single plate, L0 = initial heigh, 


import numpy as np
from numpy import log, exp
from math import sqrt, sin, cos
from M import *
from C import *
from N import *

def murray_controller(q, dq, qd, dqd, ddqd, d, hp, m, L0, kp=100, kv=100, ks=100):
    M = get_M(q, m, L0)
    C = get_C(q, dq, m, L0)
    N = get_N(q, m, L0)
    # gain matrices
    Kp = np.diag([kp, kp, kp, kp, ks, kp, kp, kp, kp])
    Kv = np.diag([kv, kv, kv, kv, ks, kv, kv, kv, kv])
    # print(f"error: {}")
    # calculate force vector
    # u = M.dot((ddqd - Kv.dot((dq-dqd)) - Kp.dot((q-dq)))) + C.dot(dq) + N
    # print(f"M matrix: {M}\n")
    # print(f"C matrix: {C}\n")
    # print(f"N matrix: {N}\n")
    feedback = M.dot(-Kv.dot(dq-dqd) - Kp.dot(q-qd))
    feed_forward = M.dot(ddqd) + C.dot(dq) + N
    # print(f"feedback term: {feedback}\n")
    # print(f"feed forward: {feed_forward}\n")
    u = feed_forward + feedback
    # u = feedback
    # print(f"our u, a.k.a forces applied to 9 params: {u}\n")
    # calculate cable forces (torque we input into motors)
    j_orientation = np.matrix([[0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]])
    # j_orientation = np.matrix([[1, 0, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0]])
    dx = d * sin(60 * np.pi/180)
    dy = d * sin(30 * np.pi/180)
    # print(f"our dx: {dx}\n")
    # print(f"our dy: {dy}\n")
    mod_force = np.matrix([[dy, dy, -d], [-dx, dx, 0], [1, 1, 1]])
    # mod_force = np.matrix([[dx, dx, -d], [-dy, dy, 0], [1, 1, 1]])
    # At = np.diag([1, 1, 1, 1, -1, 1, 1, 1, 1])
    At = np.array([[-1.0, 0, 0],
    [0, -1.0, 0],
    [0, 0, -1.0]])
    mod_force = np.matmul(At, mod_force)
    A = np.matmul(j_orientation,mod_force)
    
    # A = np.matmul(At, A)
    # print(f"Our A: {A}\n")
    # test = A * np.array([580, 580, 580]).reshape(-1,1)
    # print(f"what q would be if we multiplied with equal F1 F2 F3: {test}\n")
    # A = np.matmul(At,A)
    # print(f"Our A Matrix: {A}\n")
    tau = np.matmul(np.linalg.pinv(A), u)
    return tau


# def grab_q_murray(l1, s):
#     # phi1 = math.atan((math.sqrt(3)/3) * (l1[2] + l1[1] - 2 * l1[0])/(l1[1] - l1[2]))
#     # phi1 = math.atan2((math.sqrt(3)/3) * (l1[2] + l1[1] - 2 * l1[0]), (l1[1] - l1[2]))
#     # k1 = 2 * math.sqrt(l1[0]**2 + l1[1]**2 + l1[2]**2 - (l1[0]*l1[1]) - (l1[1] * l1[2]) - (l1[0]*l1[2]))/(d* (l1[0] + l1[1] + l1[2]))
#     temp1 = l1[0]
#     temp2 = l1[1]
#     temp3 = l1[2]
#     # l1_0 = [0.08,0.080, 0.079]
#     l1 = [temp1, temp2, temp3]
#     phi1 = math.atan2(((math.sqrt(3)/3) * (l1[2] + l1[1] - 2 * l1[0])),(l1[1] - l1[2]))
#     # phi1 = math.atan((math.sqrt(3)/3) * (l1[2] + l1[1] - 2 * l1[0])/(l1[1] - l1[2]))
#     # k1 = 2 * math.sqrt(l1[0]**2 + l1[1]**2 + l1[2]**2 - (l1[0]*l1[1]) - (l1[1] * l1[2]) - (l1[0]*l1[2]))/(d* (l1[0] + l1[1] + l1[2]))
#     # k1 = 2 * math.sqrt(l1[2]**2 + l1[0]**2 + l1[1]**2 - (l1[2]*l1[0]) - (l1[0] * l1[1]) - (l1[2]*l1[1]))/(d* (l1[2] + l1[0] + l1[1]))
#     k1 = 2 * math.sqrt(l1[2]**2 + l1[0]**2 + l1[1]**2 - (l1[2]*l1[0]) - (l1[0] * l1[1]) - (l1[2]*l1[1]))/(3* d/(l1_0[2] + l1_0[0] + l1_0[1]))
   
#     s_curr = (l1[0] + l1[1] + l1[2])/3
#     print(f"s_curr: {s_curr}")
#     # Testing new curvature equation from section 5.1.2-Determination of Curvature in "Kinematics and the Implementation of an Elephant's Trunk Manipulator and Other Continuum Style Robots"
#     # treat the whole module as single segment
#     omega = d       # radius of the segment
#     hh = s/2        # distance from the center of the segment's joint to the end of the segment
#     rr = np.sqrt(omega**2 + hh**2)
#     l2 = s_curr     # since we're treating the whole mod as a segment, take average of all cable lengths
#     gamma = np.pi - 2 * math.atan(omega/hh) - math.atan(np.sqrt(4 * rr**4 - (2 * rr**2 - l2**2)**2)/(2 * rr**2 - l2**2))
#     n = 5
#     # k1 = 2 * n * gamma/s
    
#     kx = (k1/4) * cos(phi1) 
#     ky = (k1/4) * sin(phi1)
#     # print(f"our phi: {phi1}")
#     # print(f"our kappa: {k1}")
#     # print(f"cos(phi): {cos(phi1)}\n")
#     # print(f"sin(phi): {sin(phi1)}\n")
#     # print(f"kx: {kx}\n")
#     # print(f"ky: {ky}\n")
#     theta5 =  s_curr - s
#     q = np.array([ky, kx, ky, kx, theta5, ky, kx, ky, kx]).reshape(-1,1)
#     # q = np.array([kx, ky, kx, ky, theta5, kx, ky, kx, ky]).reshape(-1,1)
#     return q


# s = (0.081 +  0.080 +  0.079)/3
# zero =  np.zeros((9,1))
# q = grab_q_murray([0.081, 0.080, 0.079], s)
# # q = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape(-1,1)
# # q = grab_q_murray([0.031, 0.030, 0.029], s)
# print(f"current q: {q}\n")

# qd = grab_q_murray([0.081, 0.030, 0.079], s)
# # qd = grab_q_murray([0.081, 0.080, 0.079], s)
# # qd = np.array([0, 0, 0, 0, -0.05, 0, 0, 0, 0]).reshape(-1,1)
# print(f"our desired: {qd}\n")
# tau_cables = murray_controller(q, zero, qd, zero, zero, d, hp, m, s)
# print(f"tau: {tau_cables.reshape(1,-1)}\n")
# tau_cables = np.maximum(tau_cables,np.array([[-30],[-30],[-30]]))

# print(f"what we will input: {tau_cables}\n")
# nnn = get_N(qd, m, s)
# print(f"gravity of qd: {nnn}\n")