import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt

msf_veh = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)
zup_state = np.array(pd.read_csv("data/tmp/zup_debug_state.csv"))

half_len = int(zup_state.shape[0] / 3)

zup_state_gx = zup_state[np.abs(zup_state[:, 1]) < 0.1, 1]
zup_state_gy = zup_state[np.abs(zup_state[:, 2]) < 0.1, 2]
zup_state_gz = zup_state[np.abs(zup_state[:, 3]) < 0.1, 3]

mean_x_l = np.mean(zup_state_gx[1:half_len])
mean_y_l = np.mean(zup_state_gy[1:half_len])
mean_z_l = np.mean(zup_state_gz[1:half_len])
mean_x_r = np.mean(zup_state_gx[half_len:-1])
mean_y_r = np.mean(zup_state_gy[half_len:-1])
mean_z_r = np.mean(zup_state_gz[half_len:-1])


mean_x = np.mean(zup_state[np.abs(zup_state[:, 1]) < 0.1, 1])
mean_y = np.mean(zup_state[np.abs(zup_state[:, 2]) < 0.1, 2])
mean_z = np.mean(zup_state[np.abs(zup_state[:, 3]) < 0.1, 3])
print(mean_x_l)
print(mean_x)
print(mean_x_r)
print(mean_y_l)
print(mean_y)
print(mean_y_r)
print(mean_z_l)
print(mean_z)
print(mean_z_r)

SUBNUM = 5
plt.subplots(SUBNUM, 1, sharex=True)
plt.subplot(SUBNUM, 1, 1)
plt.scatter(zup_state[:, 0], zup_state[:, 1], s=0.3)
plt.axhline(mean_x_l, c="red")
plt.axhline(mean_x_r, c="blue")
plt.axhline(mean_x, c="black")
plt.legend(["gyro x", "mean l", "mean r", "mean"])
# plt.ylim([-0.2, 0.2])
plt.subplot(SUBNUM, 1, 2)
plt.scatter(zup_state[:, 0], zup_state[:, 2], s=0.3)
plt.axhline(mean_y_l, c="red")
plt.axhline(mean_y_r, c="blue")
plt.axhline(mean_y, c="black")
plt.legend(["gyro y", "mean l", "mean r", "mean"])
# plt.ylim([-0.2, 0.2])
plt.subplot(SUBNUM, 1, 3)
plt.scatter(zup_state[:, 0], zup_state[:, 3], s=0.3)
plt.axhline(mean_z_l, c="red")
plt.axhline(mean_z_r, c="blue")
plt.axhline(mean_z, c="black")
plt.legend(["gyro z", "mean l", "mean r", "mean"])
# plt.ylim([-0.2, 0.2])
plt.subplot(SUBNUM, 1, 4)
plt.scatter(zup_state[:, 0], zup_state[:, 10], s=0.3)
plt.legend("msf vel")
# plt.ylim([0, 0.4])
plt.subplot(SUBNUM, 1, 5)
plt.scatter(msf_veh[:, 0], msf_veh[:, 67], s=0.3)
plt.legend("manu state")
# plt.ylim([0, 0.4])
plt.show()
