import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt

idx = 500

veh_state = np.array(pd.read_csv("data/tmp/imu_debug_state.csv"))
t = veh_state[:, 0]
veh_state = veh_state[:, 1:]

plt.figure("gyro")
plt.subplot(3, 1, 1)
gyro_x_mean = veh_state[idx:-1:100, 0].mean()
plt.text(t[2000], 0.02, "gyro x steady state: " + str(gyro_x_mean))
plt.plot(t, veh_state[:, 0] - gyro_x_mean)
plt.plot(t, veh_state[:, 6], c="black")
plt.plot(t, -veh_state[:, 6], c="black")
plt.ylim([-0.03, 0.03])
plt.subplot(3, 1, 2)
gyro_y_mean = veh_state[idx:-1:100, 1].mean()
plt.text(t[2000], 0.02, "gyro y steady state: " + str(gyro_y_mean))
plt.plot(t, veh_state[:, 1] - gyro_y_mean)
plt.plot(t, veh_state[:, 7], c="black")
plt.plot(t, -veh_state[:, 7], c="black")
plt.ylim([-0.03, 0.03])
plt.subplot(3, 1, 3)
gyro_z_mean = veh_state[idx:-1:100, 2].mean()
plt.text(t[2000], 0.02, "gyro z steady state: " + str(gyro_z_mean))
plt.plot(t, veh_state[:, 2] - gyro_z_mean)
plt.plot(t, veh_state[:, 8], c="black")
plt.plot(t, -veh_state[:, 8], c="black")
plt.ylim([-0.03, 0.03])

plt.figure("acc")
plt.subplot(3, 1, 1)
acc_x_mean = veh_state[idx:-1:100, 3].mean()
plt.text(t[2000], 0.02, "acc x steady state: " + str(acc_x_mean))
plt.plot(t, veh_state[:, 3] - acc_x_mean)
plt.plot(t, veh_state[:, 9], c="black")
plt.plot(t, -veh_state[:, 9], c="black")
plt.ylim([-0.03, 0.03])
plt.subplot(3, 1, 2)
acc_y_mean = veh_state[idx:-1:100, 4].mean()
plt.text(t[2000], 0.02, "acc y steady state: " + str(acc_y_mean))
plt.plot(t, veh_state[:, 4] - acc_y_mean)
plt.plot(t, veh_state[:, 10], c="black")
plt.plot(t, -veh_state[:, 10], c="black")
plt.ylim([-0.03, 0.03])
plt.subplot(3, 1, 3)
acc_z_mean = veh_state[idx:-1:100, 5].mean()
plt.text(t[2000], 0.02, "acc z steady state: " + str(acc_z_mean))
plt.plot(t, veh_state[:, 5] - acc_z_mean)
plt.plot(t, veh_state[:, 11], c="black")
plt.plot(t, -veh_state[:, 11], c="black")
plt.ylim([-0.03, 0.03])
plt.show()
