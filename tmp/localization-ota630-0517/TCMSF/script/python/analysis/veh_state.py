import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


veh_state = np.array(pd.read_csv("data/tmp/veh_debug_state.csv"), dtype=float)

VEH_SUBPLOTS = 4
plt.subplots(VEH_SUBPLOTS, 1, sharex=True)
plt.subplot(VEH_SUBPLOTS, 1, 1)
pitch_mean = veh_state[-1000:-1:100, 1].mean()
plt.text(veh_state[1000, 0], 0.5, "pitch steady state: " + str(pitch_mean))
plt.scatter(veh_state[:, 0], veh_state[:, 1] - pitch_mean, s=0.1)
plt.plot(veh_state[:, 0], veh_state[:, 4], c="black")
plt.plot(veh_state[:, 0], -veh_state[:, 4], c="black")
plt.ylim([-1.0, 1.0])
plt.legend(["pitch - ss mean"])
plt.subplot(VEH_SUBPLOTS, 1, 2)
spd_scale = veh_state[-1000:-1:100, 2].mean()
plt.text(veh_state[1000, 0], 0.0025, "spd_scale steady state: " + str(spd_scale))
plt.scatter(veh_state[:, 0], veh_state[:, 2] - spd_scale, s=0.1)
plt.plot(veh_state[:, 0], veh_state[:, 5], c="black")
plt.plot(veh_state[:, 0], -veh_state[:, 5], c="black")
plt.ylim([-0.005, 0.005])
plt.legend(["spd_scale - ss mean"])
plt.subplot(VEH_SUBPLOTS, 1, 3)
yaw_mean = veh_state[-1000:-1:100, 3].mean()
plt.text(veh_state[1000, 0], 0.5, "yaw steady state: " + str(yaw_mean))
plt.scatter(veh_state[:, 0], veh_state[:, 3] - yaw_mean, s=0.1)
plt.plot(veh_state[:, 0], veh_state[:, 6], c="black")
plt.plot(veh_state[:, 0], -veh_state[:, 6], c="black")
plt.ylim([-1.0, 1.0])
plt.legend(["yaw - ss mean"])
plt.subplot(VEH_SUBPLOTS, 1, 4)
plt.plot(veh_state[:, 0], veh_state[:, 9])
plt.legend(["slip index"])


plt.subplots(8, 1, sharex=True)
plt.subplot(8, 1, 1)
plt.plot(veh_state[:, 0], veh_state[:, 21])
plt.legend(["continous gnss fix"])
plt.subplot(8, 1, 2)
plt.scatter(veh_state[:, 0], veh_state[:, 22], s=0.1)
plt.legend(["align_type"])
plt.subplot(8, 1, 3)
plt.scatter(veh_state[:, 0], veh_state[:, 23], s=0.1)
plt.legend(["rtk_status"])
plt.subplot(8, 1, 4)
plt.plot(veh_state[:, 0], veh_state[:, 24])
plt.legend(["slip_index"])
plt.subplot(8, 1, 5)
plt.scatter(veh_state[:, 0], veh_state[:, 25], s=0.1)
plt.legend(["maneuver_status_by_imu"])
plt.subplot(8, 1, 6)
plt.plot(veh_state[:, 0], veh_state[:, 26])
plt.legend(["gnss_inno_2d"])
plt.subplot(8, 1, 7)
plt.scatter(veh_state[:, 0], veh_state[:, 27], s=0.1)
plt.legend(["veh_state_estimate_"])
plt.subplot(8, 1, 8)
plt.scatter(veh_state[:, 0], veh_state[:, 28], s=0.1)
plt.legend(["veh_low_dynamic_"])

plt.figure("slip")
plt.subplot(3, 1, 1)
plt.plot(veh_state[:, 0], veh_state[:, 7])
plt.subplot(3, 1, 2)
plt.plot(veh_state[:, 0], veh_state[:, 8])
plt.subplot(3, 1, 3)
plt.plot(veh_state[:, 0], veh_state[:, 9])
# plt.show()

plt.subplots(2, 1, sharex=True)
plt.subplot(2, 1, 1)
plt.plot(veh_state[:, 0], veh_state[:, 17])
plt.plot(veh_state[:, 0], veh_state[:, 18])
plt.subplot(2, 1, 2)
plt.plot(veh_state[:, 0], veh_state[:, 2])
# plt.show()

plt.figure()
plt.scatter(veh_state[:, 0], veh_state[:, 29], s=0.1)
plt.legend(["slip angle"])
plt.show()
