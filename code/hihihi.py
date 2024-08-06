# from datetime import datetime
# import os
# import numpy as np
# import scipy
# import json
# import time
# from dyn_functions import *
# from scipy.ndimage import uniform_filter1d

# y = uniform_filter1d([1.0,2.0,3.0,4.0,5.0,6.0, 7.0, 8.0, 9.0, 10.0], size=10)
# print(y)

# y = uniform_filter1d([11.0,2.0,3.0,4.0,5.0,6.0, 7.0, 8.0, 9.0, 10.0], size=10)
# print(y)

# q = np.array([[ 7.18796771e-01],
#  [-9.32767400e-03],
#  [ 1.43882999e-03],
#  [ 7.47764501e-03],
#  [-1.20264094e+00],
#  [ 3.23694783e-03],
#  [ 5.71799507e-04],
#  [-2.12957440e-02],
#  [ 2.94524311e-01],
#  [ 1.15161484e-02],
#  [ 1.58129846e-03],
#  [-7.22811747e-03],
#  [ 1.45728175e-01],
#  [ 2.91076793e-03],
#  [ 2.97862460e-04],
#  [-1.43335165e-02]])
# print(q.shape)

# q_data = np.repeat(q, 10, axis=1)

# print(q_data.shape)
# print(q_data[:, 2])

# y = uniform_filter1d(q_data[:, -10:], size=10)
# print(y[:, [5]])