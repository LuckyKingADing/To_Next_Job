import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


imu_state = np.array(pd.read_csv("data/tmp/imu_debug_state.csv"))
zup_state = np.array(pd.read_csv("data/tmp/zup_debug_state.csv"))

plt.figure("gyro")
plt.subplot(3, 1, 1)
pitch_mean = imu_state[-2000:-1:100, 1].mean()
plt.text(imu_state[2000, 0], 0.01, "pitch steady state: " + str(pitch_mean))
plt.plot(imu_state[:, 0], imu_state[:, 1] - pitch_mean)
plt.plot(imu_state[:, 0], imu_state[:, 7], c="black")
plt.plot(imu_state[:, 0], -imu_state[:, 7], c="black")
plt.ylim([-0.02, 0.02])
plt.legend(["pitch - ss mean"])
plt.subplot(3, 1, 2)
roll_mean = imu_state[-2000:-1:100, 2].mean()
plt.text(imu_state[2000, 0], 0.01, "roll steady state: " + str(roll_mean))
plt.plot(imu_state[:, 0], imu_state[:, 2] - roll_mean)
plt.plot(imu_state[:, 0], imu_state[:, 8], c="black")
plt.plot(imu_state[:, 0], -imu_state[:, 8], c="black")
plt.ylim([-0.02, 0.02])
plt.legend(["roll - ss mean"])
plt.subplot(3, 1, 3)
yaw_mean = imu_state[-2000:-1:100, 3].mean()
plt.text(imu_state[2000, 0], 0.01, "yaw steady state: " + str(yaw_mean))
plt.plot(imu_state[:, 0], imu_state[:, 3] - yaw_mean)
plt.plot(imu_state[:, 0], imu_state[:, 9], c="black")
plt.plot(imu_state[:, 0], -imu_state[:, 9], c="black")
plt.ylim([-0.02, 0.02])
plt.legend(["yaw - ss mean"])

plt.figure("acc")
plt.subplot(3, 1, 1)
x_mean = imu_state[-1000:-1:100, 4].mean()
plt.text(imu_state[2000, 0], 0.025, "x steady state: " + str(x_mean))
plt.plot(imu_state[:, 0], imu_state[:, 4] - x_mean)
plt.scatter(zup_state[:, 0], zup_state[:, 10] / 4.0, c="red", s=3)
plt.plot(imu_state[:, 0], imu_state[:, 10], c="black")
plt.plot(imu_state[:, 0], -imu_state[:, 10], c="black")
plt.ylim([-0.05, 0.05])
plt.legend(["x - ss mean"])
plt.subplot(3, 1, 2)
y_mean = imu_state[-1000:-1:100, 5].mean()
plt.text(imu_state[2000, 0], 0.025, "y steady state: " + str(y_mean))
plt.plot(imu_state[:, 0], imu_state[:, 5] - y_mean)
plt.plot(imu_state[:, 0], imu_state[:, 11], c="black")
plt.plot(imu_state[:, 0], -imu_state[:, 11], c="black")
plt.ylim([-0.05, 0.05])
plt.legend(["y - ss mean"])
plt.subplot(3, 1, 3)
z_mean = imu_state[-1000:-1:100, 6].mean()
plt.text(imu_state[2000, 0], 0.025, "z steady state: " + str(z_mean))
plt.plot(imu_state[:, 0], imu_state[:, 6] - z_mean)
plt.plot(imu_state[:, 0], imu_state[:, 12], c="black")
plt.plot(imu_state[:, 0], -imu_state[:, 12], c="black")
plt.ylim([-0.05, 0.05])
plt.legend(["z - ss mean"])
plt.show()
