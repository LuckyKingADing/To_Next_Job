import pandas as pd
import numpy as np
import math

from scipy.optimize import minimize

import scipy.optimize as otm

from matplotlib import pyplot as plt
from scipy.signal import correlate


def InterpState(state, t, t_ref):
    state_ = np.zeros([t_ref.shape[0], state.shape[1]])
    for i in range(state.shape[1]):
        state_[:, i] = np.interp(t_ref, t, state[:, i])
    return state_


tc = np.array(pd.read_csv("data/tmp/msf_debug_state.csv"))
rts = np.array(pd.read_csv("data/tmp/rts_result.csv"))

rts_itp = rts
# rts_itp = InterpState(rts, rts[:, 0], tc[:, 0])

plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.scatter(rts_itp[:, 0], rts_itp[:, 1] * 180.0 / np.pi, s=0.3)
plt.scatter(tc[:, 0], tc[:, 4], s=0.3)
plt.legend(["RTS pitch", "TCMSF pitch"])
plt.subplot(3, 1, 2)
plt.scatter(rts_itp[:, 0], rts_itp[:, 2] * 180.0 / np.pi, s=0.3)
plt.scatter(tc[:, 0], tc[:, 5], s=0.3)
plt.legend(["RTS roll", "TCMSF roll"])
plt.subplot(3, 1, 3)
plt.scatter(rts_itp[:, 0], rts_itp[:, 3] * 180.0 / np.pi, s=0.3)
plt.scatter(tc[:, 0], tc[:, 6], s=0.3)
plt.legend(["RTS yaw", "TCMSF yaw"])

plt.subplots(3, 1, sharex=True)
plt.subplot(3, 1, 1)
plt.scatter(rts_itp[:, 0], rts_itp[:, 4], s=0.3)
plt.scatter(tc[:, 0], tc[:, 7], s=0.3)
plt.legend(["RTS velE", "TCMSF velE"])
plt.subplot(3, 1, 2)
plt.scatter(rts_itp[:, 0], rts_itp[:, 5], s=0.3)
plt.scatter(tc[:, 0], tc[:, 8], s=0.3)
plt.legend(["RTS velN", "TCMSF velN"])
plt.subplot(3, 1, 3)
plt.scatter(rts_itp[:, 0], rts_itp[:, 6], s=0.3)
plt.scatter(tc[:, 0], tc[:, 9], s=0.3)
plt.legend(["RTS velU", "TCMSF velU"])


plt.show()
