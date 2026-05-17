import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt


def InterpState(state, t, t_ref):
    state_ = np.zeros([t_ref.shape[0], state.shape[1]])
    for i in range(state.shape[1]):
        state_[:, i] = np.interp(t_ref, t, state[:, i])
    return state_


def YawChangeIdx(yaw):
    dyaw = np.abs(yaw[1:] - yaw[0:-1])
    return np.where(dyaw > 180.0)[0] + 1


def MakeYawContinous(yaw):
    MsfYawChange = YawChangeIdx(yaw)
    print(MsfYawChange)
    for idx in MsfYawChange:
        yaw[idx:] = yaw[idx:] - (yaw[idx] - yaw[idx - 1]) + (yaw[idx + 1] - yaw[idx])


slp_state = np.array(pd.read_csv("data/tmp/slp_debug_state.csv"))
msf = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)
gnss = np.array(pd.read_csv("data/tmp/gps_debug_state.csv"), dtype=float)


msf_interp = InterpState(msf, msf[:, 0], gnss[:, 0])

MakeYawContinous(gnss[:, 7])
MakeYawContinous(msf_interp[:, 6])

high_speed_ = msf_interp[:, 53] > 10.0

SUBPLOTS = 4
plt.subplots(SUBPLOTS, 1, sharex=True)
plt.subplot(SUBPLOTS, 1, 1)
plt.scatter(slp_state[:, 0], slp_state[:, 5], s=0.3)
plt.ylim([-2, 2])
plt.legend(["slip index"])
plt.subplot(SUBPLOTS, 1, 2)
slip_angle = slp_state[:, 6] * 180 / np.pi
plt.scatter(slp_state[:, 0], slip_angle, s=0.3)
plt.ylim([-2, 2])
plt.legend(["slip angle"])
plt.subplot(SUBPLOTS, 1, 3)
plt.scatter(slp_state[:, 0], slp_state[:, 5], s=0.3)
plt.scatter(slp_state[:, 0], slip_angle * 3.2, s=0.3)
plt.axhline(0, c="black")
plt.ylim([-2, 2])
plt.legend(["slip index", "slip angle"])
plt.subplot(SUBPLOTS, 1, 4)
plt.scatter(
    gnss[high_speed_, 0],
    (msf_interp[high_speed_, 6] + gnss[high_speed_, 7] * 180 / np.pi) * 3.2,
    s=0.3,
)
plt.scatter(slp_state[:, 0], slp_state[:, 5], s=0.3)
plt.axhline(0, c="black")
plt.ylim([-2, 2])
plt.legend(["diff hdg"])
plt.show()

# plt.figure()
# plt.scatter(gnss[high_speed_, 0], msf_interp[high_speed_, 6], s=0.3)
# plt.scatter(gnss[high_speed_, 0], -gnss[high_speed_, 7] * 180 / np.pi, s=0.3)
# plt.legend(["msf hdg", "gnss hdg"])
# plt.show()
