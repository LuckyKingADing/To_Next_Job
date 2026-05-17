import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt

plt.rcParams["legend.loc"] = "upper right"

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


msf = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"), dtype=float)

FIG_NUM = 5
plt.subplots(FIG_NUM,1,sharex=True)
plt.subplot(FIG_NUM,1,1)
plt.plot(msf[:,0],msf[:,53])
plt.legend(["vel norm"])

plt.subplot(FIG_NUM,1,2)
plt.plot(msf[:,0],msf[:,53])
plt.axhline(y=0.1,color="black")
plt.ylim([-0.02,0.2])
plt.legend(["vel norm"])


plt.subplot(FIG_NUM,1,3)
plt.plot(msf[:,0],msf[:,48])
plt.axhline(y=0.1,color="black")
plt.ylim([-0.02,0.2])
plt.legend(["vel right"])

plt.subplot(FIG_NUM,1,4)
plt.plot(msf[:,0],msf[:,49])
plt.axhline(y=0.1,color="black")
plt.ylim([-0.02,0.2])
plt.legend(["vel front"])

plt.subplot(FIG_NUM,1,5)
plt.plot(msf[:,0],msf[:,9])
plt.axhline(y=0.1,color="black")
plt.ylim([-0.02,0.2])
plt.legend(["vel up"])
plt.show()